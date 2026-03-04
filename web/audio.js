class NuPSXAudioProcessor extends AudioWorkletProcessor {
    constructor() {
        super();
        this.queue = [];
        this.offset = 0;
        this.port.onmessage = (e) => {
            this.queue.push(e.data);
            
            if (this.queue.length > 10) {
                this.queue.shift();
            }
        };
    }

    process(inputs, outputs) {
        const left  = outputs[0][0];
        const right = outputs[0][1];
        const n = left.length;
        let written = 0;

        while (written < n && this.queue.length > 0) {
            const { l, r } = this.queue[0];
            const avail = l.length - this.offset;
            const count = Math.min(avail, n - written);

            left.set(l.subarray(this.offset, this.offset + count), written);
            right.set(r.subarray(this.offset, this.offset + count), written);

            written += count;
            this.offset += count;

            if (this.offset >= l.length) {
                this.queue.shift();
                this.offset = 0;
            }
        }

        for (let i = written; i < n; i++) {
            left[i] = 0;
            right[i] = 0;
        }

        return true;
    }
}

registerProcessor('nupsx-audio', NuPSXAudioProcessor);
