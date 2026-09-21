/* Exercise the installed ST VS Code debug adapter through standard DAP.
 * Requires an explicit --flash and a verified local backup before downloading.
 * The extension and its server binaries are not copied into this repository.
 */
const fs = require("node:fs");
const path = require("node:path");
const os = require("node:os");
const crypto = require("node:crypto");
const {spawn, execFileSync} = require("node:child_process");
const root = path.resolve(__dirname, "..");
const options = process.argv.slice(2);
const value = (key, fallback) => options.includes(key) ? options[options.indexOf(key)+1] : fallback;
const flash = options.includes("--flash");
const probe = value("--probe", "");
const seconds = Number(value("--seconds", "35"));
if (!/^[a-zA-Z0-9_-]+$/.test(probe) || !Number.isFinite(seconds) || seconds<1 || seconds>1200)
    throw Error("Provide --probe SERIAL and --seconds 1..1200");
const config = JSON.parse(fs.readFileSync(path.join(root,".local/tools.json")));
// ST's server binds all interfaces. Verify deny-only firewall protection BEFORE launch.
execFileSync("powershell.exe", ["-NoProfile", "-File", path.join(root,"Tools/debug-network.ps1")], {windowsHide:true, stdio:"inherit"});
const ext = path.join(os.homedir(), ".vscode/extensions");
const adapter = path.join(ext, "stmicroelectronics.stm32cube-ide-debug-stlink-gdbserver-1.4.0/lib/adapter/STLinkDebugTargetAdapter.js");
const cube = path.join(ext, "stmicroelectronics.stm32cube-ide-core-1.4.0-win32-x64/resources/binaries/win32/x86_64");
const elf = path.resolve(root,value("--elf","build/Diagnostics-local/RFID_CC1101_V38.elf"));
const bytes = fs.readFileSync(elf);
if(bytes[4]!==1 || bytes[5]!==1) throw Error("Expected little-endian ELF32");
for(let p=bytes.readUInt32LE(28),i=0;i<bytes.readUInt16LE(44);++i,p+=bytes.readUInt16LE(42)) {
    if(bytes.readUInt32LE(p)!==1) continue;
    const address=bytes.readUInt32LE(p+12), size=bytes.readUInt32LE(p+16);
    if(size && (address<0x08000000 || address+size>0x08010000)) throw Error("ELF load range is not internal Flash");
}
const hash = b => crypto.createHash("sha256").update(b).digest("hex").toUpperCase();
const backups = path.join(root,".local/backups");
const matches = fs.readdirSync(backups).filter(n=>n.endsWith("-"+probe)).sort().reverse();
let backup;
for(const dir of matches) {
    const file=path.join(backups,dir,"manifest.json");
    if(!fs.existsSync(file)) continue;
    const m=JSON.parse(fs.readFileSync(file));
    const base=path.dirname(file);
    if(m.Complete && m.ProbeSerial===probe && m.OptionsSHA256 &&
       hash(fs.readFileSync(path.join(base,"flash.bin")))===m.FlashSHA256 &&
       hash(fs.readFileSync(path.join(base,"eeprom.bin")))===m.EepromSHA256 &&
       hash(fs.readFileSync(path.join(base,"options.bin")))===m.OptionsSHA256) { backup=base; break; }
}
if(!backup) throw Error("No complete, hash-verified backup for this probe");
const legacyReference = fs.readFileSync(path.resolve(root,
    value("--legacy-reference", path.join(backup,"eeprom.bin"))));
if(legacyReference.length!==2048 || !legacyReference.subarray(0,8).equals(
    fs.readFileSync(path.join(backup,"eeprom.bin")).subarray(0,8)))
    throw Error("Legacy reference identity does not match the verified backup");
const stamp=new Date().toISOString().replace(/[:.]/g,"-");
const reportDir=path.join(root,".local/debug",stamp);
fs.mkdirSync(reportDir,{recursive:true});
const log=fs.createWriteStream(path.join(reportDir,"dap.jsonl"));
const env={...process.env, PATH:[cube,path.join(config.CubeClt,"GNU-tools-for-STM32/bin"),process.env.PATH].join(path.delimiter)};
const child=spawn(process.execPath,[adapter],{cwd:root,env,windowsHide:true});
let buffer=Buffer.alloc(0), sequence=0;
const pending=new Map(), queued=[], watchers=[];
child.stderr.on("data",b=>{ log.write(JSON.stringify({stderr:b.toString()})+"\n"); process.stderr.write(b); });
function dispatch(message) {
    log.write(JSON.stringify(message)+"\n");
    if(message.type==="response") {
        const waiter=pending.get(message.request_seq);
        if(waiter) { clearTimeout(waiter.timer); pending.delete(message.request_seq);
            message.success ? waiter.resolve(message.body) : waiter.reject(Error(message.message||JSON.stringify(message))); }
    } else if(message.type==="event") {
        if(message.event==="output" && message.body?.output) process.stdout.write(message.body.output);
        const index=watchers.findIndex(w=>w.event===message.event);
        if(index>=0) { const w=watchers.splice(index,1)[0]; clearTimeout(w.timer); w.resolve(message.body); }
        else queued.push(message);
    } else if(message.type==="request") {
        const reply={seq:++sequence,type:"response",request_seq:message.seq,command:message.command,success:false,message:"No interactive client action allowed"};
        const b=Buffer.from(JSON.stringify(reply)); child.stdin.write("Content-Length: "+b.length+"\r\n\r\n"); child.stdin.write(b);
    }
}
child.stdout.on("data",chunk=>{
    buffer=Buffer.concat([buffer,chunk]);
    for(;;) {
        const split=buffer.indexOf("\r\n\r\n"); if(split<0) break;
        const header=buffer.subarray(0,split).toString();
        const match=/Content-Length: (\d+)/i.exec(header);
        if(!match) throw Error("Invalid DAP framing: "+header);
        const n=Number(match[1]); if(buffer.length<split+4+n) break;
        const payload=buffer.subarray(split+4,split+4+n);
        buffer=buffer.subarray(split+4+n); dispatch(JSON.parse(payload));
    }
});
function request(command,args={},timeout=120000) {
    const seq=++sequence, b=Buffer.from(JSON.stringify({seq,type:"request",command,arguments:args}));
    return new Promise((resolve,reject)=>{
        const timer=setTimeout(()=>{pending.delete(seq);reject(Error("DAP timeout: "+command));},timeout);
        pending.set(seq,{resolve,reject,timer}); child.stdin.write("Content-Length: "+b.length+"\r\n\r\n"); child.stdin.write(b);
    });
}
function event(name,timeout=120000) {
    const index=queued.findIndex(q=>q.event===name);
    if(index>=0) return Promise.resolve(queued.splice(index,1)[0].body);
    return new Promise((resolve,reject)=>{
        const w={event:name,resolve,timer:setTimeout(()=>{const i=watchers.indexOf(w);if(i>=0)watchers.splice(i,1);reject(Error("DAP event timeout: "+name));},timeout)};
        watchers.push(w);
    });
}
const delay = ms => new Promise(r=>setTimeout(r,ms));
(async()=>{
    let connected=false;
    try {
        await request("initialize",{clientID:"v38-smoke",adapterID:"stlinkgdbtarget",linesStartAt1:true,columnsStartAt1:true,pathFormat:"path",supportsVariableType:true,supportsRunInTerminalRequest:false});
        const launch = request("launch",{
            type:"stlinkgdbtarget",request:"launch",name:"V3.8 hardware smoke",origin:"snippet",cwd:root,
            gdb:path.join(config.CubeClt,"GNU-tools-for-STM32/bin/arm-none-eabi-gdb.exe"),
            program:elf,deviceName:"STM32L051C8T6",serverHost:"localhost",serverPort:"61234",
            serverExe:"ST-LINK_gdbserver.exe",
            serverCwd:path.join(config.CubeClt,"STLink-gdb-server/bin"),
            serverCubeProgPath:path.join(config.CubeClt,"STM32CubeProgrammer/bin"),
            serverInterface:"SWD",serverInterfaceFrequency:"100",serverSerialNumber:probe,
            serverReset:flash?"Connect under reset":"None",serverVerify:true,
            serverSemihosting:{enabled:false},serverRtos:{enabled:false},liveWatch:{enabled:false},
            deviceDebugInLowPower:flash,deviceStopWDGOnHalt:flash,runEntry:flash?"main":"",
            automaticallyKillServer:true,openGdbConsole:false,preBuild:"",
            imagesAndSymbols:[flash?{imageFileName:elf,symbolFileName:elf}:{symbolFileName:elf}]
        });
        await Promise.race([event("initialized"), launch.then(() => new Promise(() => {}))]);
        await request("configurationDone");
        await launch; connected=true;
        const addresses=execFileSync("powershell.exe",["-NoProfile","-Command","Get-NetTCPConnection -State Listen -LocalPort 61234 | Select-Object -ExpandProperty LocalAddress"],{windowsHide:true}).toString().trim().split(/\s+/);
        if(!addresses.length) throw Error("GDB listener missing");
        if(flash) {
            const stopped=await event("stopped");
            await request("continue",{threadId:stopped.threadId||1});
        }
        await delay(seconds*1000);
        queued.splice(0,queued.length);
        await request("pause",{threadId:1});
        const stopped=await event("stopped");
        const stack=await request("stackTrace",{threadId:stopped.threadId||1,startFrame:0,levels:1});
        const frameId=stack.stackFrames[0].id;
        const results={flash,elapsedSeconds:seconds,elfSha256:hash(bytes),listeners:addresses,remoteBlockedByFirewall:true};
        for(const expression of ["history.stage","history.elapsed","history.reset_count","sensor_ready","storage_ready","sensor_errors","storage_errors","radio_errors","history.current[0]","history.current[1]","history.current[2]","history.current[3]","history.current[4]","history.current[5]"])
            results[expression]=(await request("evaluate",{expression,frameId,context:"watch"})).result;
        const memory=await request("readMemory",{memoryReference:"0x08080000",count:0x240});
        const legacy=Buffer.from(memory.data,"base64");
        if(!legacy.equals(legacyReference.subarray(0,0x240))) throw Error("Legacy EEPROM region changed");
        results.legacyRegionUnchanged=true;
        fs.writeFileSync(path.join(reportDir,"result.json"),JSON.stringify(results,null,2));
        process.stdout.write(JSON.stringify(results,null,2)+"\n");
        await request("continue",{threadId:stopped.threadId||1});
    } finally {
        try { await request("disconnect",{terminateDebuggee:false},10000); } catch(error) { process.stderr.write(error.message+"\n"); }
        await delay(500); if(child.exitCode===null) child.kill();
        for(const p of pending.values()) clearTimeout(p.timer);
        for(const w of watchers) clearTimeout(w.timer);
        log.end();
        process.stdout.write("ST debug session closed: "+reportDir+"\n");
    }
})().catch(e=>{console.error(e);process.exitCode=1;});
