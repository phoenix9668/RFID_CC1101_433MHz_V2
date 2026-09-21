/* Inspect a read-only STM32L0 data EEPROM dump without exposing device identity. */
const fs = require("node:fs");
const crypto = require("node:crypto");
const bytes = fs.readFileSync(process.argv[2]);
if (bytes.length !== 2048) throw Error("Expected a 2048-byte EEPROM image");
const crc32 = data => {
    let crc = 0xffffffff;
    for (const byte of data) {
        crc ^= byte;
        for (let i=0;i<8;i++) crc = (crc>>>1) ^ (crc&1 ? 0xedb88320 : 0);
    }
    return (~crc)>>>0;
};
const records=[];
let latest;
for(let slot=0;slot<46;slot++) {
    const r=bytes.subarray(0x240+slot*32,0x260+slot*32);
    const kind=slot<16 ? 1 : 2;
    if(r.readUInt32LE(28)!==0x38524649 || r[22]!==1 || r[23]!==kind ||
       r[20]>=12 || r[21]!==0 || r.readUInt16LE(16)>1200 ||
       r.readUInt32LE(24)!==crc32(r.subarray(0,24))) continue;
    const record={slot,kind,sequence:r.readUInt32LE(0),stage:r[20],
        elapsed:r.readUInt16LE(16),reset:r.readUInt16LE(18),
        counts:Array.from({length:6},(_,i)=>r.readUInt16LE(4+i*2))};
    records.push(record);
    if(!latest || ((record.sequence-latest.sequence)|0)>0) latest=record;
}
const result={sha256:crypto.createHash("sha256").update(bytes).digest("hex"),validRecords:records.length,latest,records};
if(process.argv[3]) {
    const before=fs.readFileSync(process.argv[3]);
    if(before.length!==2048) throw Error("Expected a 2048-byte reference image");
    result.identityUnchanged=bytes.subarray(0,8).equals(before.subarray(0,8));
    result.legacyRegionUnchanged=bytes.subarray(0,0x240).equals(before.subarray(0,0x240));
}
process.stdout.write(JSON.stringify(result,null,2)+"\n");
