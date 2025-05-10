# nuPSX

nuPSX is an experimental PlayStation 1 emulator written in Zig, built as a learning exercise to explore Zig's capabilities and PSX hardware architecture. 

Still in early development with no intention of becoming usable by the general bublic. If you are looking for the emulator to play games, check out DuckStation or PCSX-Redux.

> Made possible thanks to [zig-gamedev](https://github.com/zig-gamedev)—a collection of high quality idiomatic wrappers for graphics and audio APIs like WebGPU, OpenGL, SDL, ImGui, and more.

## Running

```
git clone https://github.com/maxpoletaev/nupsx.git
cd nupsx
zig build run -- --bios SCPH1001.bin
```

## References
 
 * [PSX-SPX Playstation Specifications](https://problemkaputt.de/psx-spx.htm) by Martin "nocash" Korth 
 * [PlayStation Emulation](https://jsgroth.dev/blog/posts/ps1-cpu/) blog post series by  James Groth
 * [PlayStation Emulation Guide](https://github.com/simias/psx-guide) ([PDF](https://github.com/simias/psx-guide/issues/4#issuecomment-181339467)) by Lionel "simias" Flandrin
