function formatWasmError(err) {
    const type = err?.name ?? 'Error';
    const msg  = err?.message ?? String(err);
    return `${type}: ${msg}`;
}

class NuPSX {
    constructor() {
        this.instance = null;
        this.memory = null;
        this.decoder = new TextDecoder('utf-8');
        this._lastLog = null;
    }

    _call(fn) {
        try {
            return fn();
        } catch (err) {
            const wasmMsg = this._lastLog ? `\nLast log: ${this._lastLog}` : '';
            alert(`nuPSX error\n${formatWasmError(err)}${wasmMsg}`);
            throw err;
        }
    }

    async load(wasmPath = 'nupsx.wasm') {
        const response = await fetch(wasmPath);
        const buffer = await response.arrayBuffer();
        const module = await WebAssembly.compile(buffer);

        const imports = {
            env: {
                js_console_log: (ptr, len) => {
                    const bytes = new Uint8Array(this.memory.buffer, ptr, len);
                    const text = this.decoder.decode(bytes);
                    this._lastLog = text;
                    console.log(text);
                }
            }
        };

        this.instance = await WebAssembly.instantiate(module, imports);
        this.memory = this.instance.exports.memory;
        return this;
    }

    alloc(size) {
        return this._call(() => this.instance.exports.alloc(size));
    }

    allocAligned(size, alignment) {
        return this._call(() => this.instance.exports.allocAligned(size, alignment));
    }

    init(biosPtr, biosLen) {
        this._call(() => this.instance.exports.init(biosPtr, biosLen));
    }

    loadExe(exePtr, exeLen) {
        this._call(() => this.instance.exports.loadExe(exePtr, exeLen));
    }

    loadBin(binPtr, binLen) {
        this._call(() => this.instance.exports.loadBin(binPtr, binLen));
    }

    runFrame() {
        this._call(() => this.instance.exports.runFrame());
    }

    getVRAM() {
        const ptr = this._call(() => this.instance.exports.getVRAMPtr());
        return new Uint16Array(this.memory.buffer, ptr, 1024 * 512);
    }

    getDisplayInfo() {
        const ptr = this._call(() => this.instance.exports.getDisplayInfoPtr());
        const mem = new Uint16Array(this.memory.buffer);
        const base = ptr / 2; // each field is u16
        return {
            offsetX: mem[base + 0],
            offsetY: mem[base + 1],
            width:   mem[base + 2],
            height:  mem[base + 3],
        };
    }

    setButtonState(state) {
        this._call(() => this.instance.exports.setButtonState(state));
    }

    static Button = {
        SELECT: 1 << 0,
        L3: 1 << 1,
        R3: 1 << 2,
        START: 1 << 3,
        UP: 1 << 4,
        RIGHT: 1 << 5,
        DOWN: 1 << 6,
        LEFT: 1 << 7,
        L2: 1 << 8,
        R2: 1 << 9,
        L1: 1 << 10,
        R1: 1 << 11,
        TRIANGLE: 1 << 12,
        CIRCLE: 1 << 13,
        CROSS: 1 << 14,
        SQUARE: 1 << 15,
    };
}

document.addEventListener('DOMContentLoaded', () => {
    const $biosFile = document.getElementById('biosFile');
    const $exeFile = document.getElementById('exeFile');
    const $binFile = document.getElementById('binFile');
    const $startBtn = document.getElementById('startBtn');
    const $setup = document.getElementById('setup');
    const $fps = document.getElementById('fps');
    const $display = document.getElementById('display');

    const ctx = $display.getContext('2d');
    let imageData = ctx.createImageData(320, 240);
    let emu = null;
    let buttonState = 0;

    const TARGET_FPS = 60;
    const FRAME_TIME = 1000 / TARGET_FPS;

    let lastFrameTime = performance.now();
    let lastTime = performance.now();
    let frameCount = 0;

    const keyMap = {
        'KeyW': NuPSX.Button.UP,
        'KeyA': NuPSX.Button.LEFT,
        'KeyS': NuPSX.Button.DOWN,
        'KeyD': NuPSX.Button.RIGHT,
        'KeyK': NuPSX.Button.CROSS,
        'KeyL': NuPSX.Button.CIRCLE,
        'KeyJ': NuPSX.Button.SQUARE,
        'KeyI': NuPSX.Button.TRIANGLE,
        'KeyE': NuPSX.Button.L1,
        'KeyQ': NuPSX.Button.L2,
        'KeyU': NuPSX.Button.R1,
        'KeyO': NuPSX.Button.R2,
        'Enter': NuPSX.Button.START,
        'ShiftRight': NuPSX.Button.SELECT,
    };

    async function init(biosData, exeData, binData) {
        emu = await new NuPSX().load();

        const biosPtr = emu.alloc(biosData.length);
        new Uint8Array(emu.memory.buffer).set(biosData, biosPtr);
        emu.init(biosPtr, biosData.length);

        if (exeData) {
            const exePtr = emu.alloc(exeData.length);
            new Uint8Array(emu.memory.buffer).set(exeData, exePtr);
            emu.loadExe(exePtr, exeData.length);
        } else if (binData) {
            const binPtr = emu.allocAligned(binData.length, 2);
            new Uint8Array(emu.memory.buffer).set(binData, binPtr);
            emu.loadBin(binPtr, binData.length);
        }

        window.addEventListener('keydown', (e) => {
            if (keyMap[e.code] !== undefined) {
                e.preventDefault();
                buttonState |= keyMap[e.code];
            }
        });

        window.addEventListener('keyup', (e) => {
            if (keyMap[e.code] !== undefined) {
                e.preventDefault();
                buttonState &= ~keyMap[e.code];
            }
        });

        requestAnimationFrame(loop);
    }

    function isInFocus() {
        return document.hasFocus() && document.visibilityState === 'visible';
    }

    function loop() {
        requestAnimationFrame(loop);

        if (!isInFocus()) return;

        const now = performance.now();
        const elapsed = now - lastFrameTime;
        if (elapsed < FRAME_TIME) return;

        lastFrameTime = now - (elapsed % FRAME_TIME);

        emu.setButtonState(buttonState);
        emu.runFrame();
        frameCount++;

        const { offsetX, offsetY, width, height } = emu.getDisplayInfo();

        if ($display.width !== width || $display.height !== height) {
            $display.width = width;
            $display.height = height;
            imageData = ctx.createImageData(width, height);
        }

        const vram = emu.getVRAM();
        const pixels = imageData.data;
        for (let row = 0; row < height; row++) {
            const vramRow = (offsetY + row) * 1024;
            const outRow = row * width;
            for (let col = 0; col < width; col++) {
                const rgb555 = vram[vramRow + offsetX + col];
                const i = (outRow + col) * 4;
                pixels[i + 0] = ((rgb555 >>  0) & 0x1f) << 3;
                pixels[i + 1] = ((rgb555 >>  5) & 0x1f) << 3;
                pixels[i + 2] = ((rgb555 >> 10) & 0x1f) << 3;
                pixels[i + 3] = 255;
            }
        }
        ctx.putImageData(imageData, 0, 0);

        if (now - lastTime >= 1000) {
            $fps.textContent = `FPS: ${frameCount}`;
            frameCount = 0;
            lastTime = now;
        }
    }

    async function readFile(input) {
        const file = input.files[0];
        if (!file) return null;
        return new Uint8Array(await file.arrayBuffer());
    }

    $startBtn.addEventListener('click', async function() {
        const biosData = await readFile($biosFile);
        if (!biosData) { alert('Please select a BIOS file.'); return; }

        const exeData = await readFile($exeFile);
        const binData = await readFile($binFile);
        if (!exeData && !binData) { alert('Please select a PS-EXE or BIN file.'); return; }

        $startBtn.disabled = true;
        await init(biosData, exeData, binData);
        $setup.style.display = 'none';
        $fps.style.display = 'block';
    });
});
