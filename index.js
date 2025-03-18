import cp from 'child_process';
import fs from 'fs';

const path = 'ttyUSB0';

console.log('using device /dev/' + path);
cp.execSync('stty -F /dev/' + path + ' 9600 raw -echo -echoe -echok -echoctl -echoke', { stdio: 'inherit' });

const port = fs.createWriteStream(`/dev/${path}`, { flags: 'w' });

const write = (data) => {
  console.log(`< ${Array.from(data).map(a => a.toString(16).padStart(2, '0')).join(' ')}`);
  if (!port) {
    console.log('(port not open)');
    return;
  }
  return new Promise(resolve => port.write(data, resolve));
}

const GET_PULSES = 0x33;
const GET_SERIAL_ENABLED = 0xf3;
const ROTATE = 0xfd;
const STOP = 0xf7;

const PULSES_PER_ROTATION = 200 * 8 * 37;// 200 * 16 * 25;

const sleep = (ms) => new Promise(resolve => setTimeout(resolve, ms));

const msgQueue = [];

let buffer = [];
const onMessage = async (msg) => {
  const data = Array.from(msg);

  while (data.length > 0) {
    buffer.push(data.shift());
  }

  if (msgQueue.length === 0) {
    console.log('< (ignored)', msg.data);
    return;
  }

  const [numBytes, resolve] = msgQueue[0];
  if (buffer.length >= numBytes) {
    const buf = buffer.slice(0, numBytes);
    buffer = buffer.slice(numBytes);
    resolve(buf);
    msgQueue.shift();
  }
}

fs.createReadStream(`/dev/${path}`, { flags: 'r', bufferSize: 1 }).on('data', onMessage);

const read = async (id, numBytes) => {
  const abort = () => {
    while (msgQueue.length) msgQueue.shift();
    throw new Error('timeout');
  }
  const t = setTimeout(abort, 3000);
  const [rid, ...res] = await new Promise(resolve => msgQueue.push([numBytes + 1, resolve]));
  clearTimeout(t);
  if (rid !== 0xe0 + id) throw new Error('invalid id');
  return res;
}

let locked = false;
const send = async (id, msg, returnLength = 1) => {
  while (locked) await sleep(100);
  locked = true;

  write(Buffer.from([0xe0 + id, ...msg]));

  const res = await read(id, returnLength);
  locked = false;
  return res;
}

// max speeed = 0x7f aka 127
const rotate = async (id, speed, position) => {
  let pos = Math.abs(Math.round(position * PULSES_PER_ROTATION / 360));
  let ok = 0;
  const signBit = (position > 0) ? 0b10000000 : 0;
  while (pos > 0) {
    const p = pos > 0xffff ? 0xffff : pos;
    pos -= p;
    console.log('rotate', pos, id, signBit, speed, p & 0xff, p >> 8);
    const [lastok] = await send(id, [
      ROTATE,
      signBit + speed,
      0xff && (p >> 8), p & 0xff
    ], 1);
    ok = lastok;
  }
  return ok === 1;
}

const checkUartEnabled = async (id) => {
  const [res] = await send(id, [GET_SERIAL_ENABLED, 0x01], 1);
  return res === 1;
}

const getPulses = async (id) => {
  const [a, b, c, d] = await send(id, [GET_PULSES], 4);
  return (a << 24) + (b << 16) + (c << 8) + d;
}

const getAngle = async (id) => {
  const pulses = await getPulses(id);
  return pulses * 360 / PULSES_PER_ROTATION;
}

const stop = async (id) => {
  await send(id, [STOP], 1);
}

await sleep(200);
const uart = await checkUartEnabled(1);
console.log('serial', uart);
const angle = await getAngle(1);
console.log('angle', angle);
//await write(Buffer.from([0xe0, GET_SERIAL_ENABLED, 0x01]));
await sleep(1000)

for (let i = 0; i < 10; i++) {
  await rotate(1, 120, 10)
  await sleep(2000)
  console.log('angle', await getAngle(1));
  await rotate(1, 120, -10)
  await sleep(2000)
  console.log('angle', await getAngle(1));
}
