# nuPSX

nuPSX is an experimental PlayStation 1 emulator and debugger written in Zig, built as a learning exercise to explore Zig's capabilities and PSX hardware architecture. Still in early development with no intention of becoming usable by the general public. If you are looking for the emulator to play games, check out DuckStation.

<p>
<img src="./images/debugger.png" width="99%">
<img src="./images/crash.webp" width="49%">
<img src="./images/spyro.webp" width="49%">
<img src="./images/mgs.webp" width="49%">
<img src="./images/rr.webp" width="49%">
</p>

## State

Can boot and play a few commercial games (notably Crash Bandicoot and MGS) and homebrew demos. Lots of things are still missing or incomplete. Tested 2D games are mostly fine, 3D games have minor graphical glitches. Sound system is not complete but functional enough to play music and most of sound effects in tested games.

|Game|Status|Note|
|-|-|-|
|Battle Arena Toshinden|🟢 Playable||
|Colin McRae Rally 2.0|🔴 Crashes||
|Crash Bandicoot|🟢 Playable||
|Crash Bandicoot 2|🟢 Playable||
|Crash Bash|🔴 Freezes||
|Crash Team Racing|🟢 Playable|Sound issues (no voice)|
|Earthworm Jim 2|🟢 Playable||
|Gran Turismo|🟢 Playable||
|Metal Gear Solid|🟢 Playable|Cutscene glitches|
|Mortal Kombat 2|🟢 Playable||
|Rayman|🔴 Freezes||
|Rayman 2|🟠 In game|Graphics and sound issues|
|Resident Evil 2|🟢 Playable||
|Ridge Racer|🟢 Playable||
|Spyro the Dragon|🟠 In game|Sound issues|
|Spyro Year of the Dragon|🔴 Title screen||
|Tekken 3|🟠 In game|Wrong screen resolution|
|Tomb Raider III|🔴 Title screen||
|Tomb Raider|🔴 Crashes||
|Wipeout|🔴 Freezes||

## Download

Prebuilt binaries are available in the [Releases](https://github.com/maxpoletaev/nupsx/releases)

## Build & Run

```
git clone https://github.com/maxpoletaev/nupsx.git
cd nupsx
zig build --release=safe
./zig-out/bin/nupsx --bios SCPH1001.bin
```

## Command Line Options

```
nuPSX - A PlayStation emulator

Usage: nupsx --bios <path> [options]

Options:
  --bios <path>        Path to BIOS file (required)
  --exe <path>         Path to executable file to run
  --cdrom <path>       Path to CD-ROM image file (.cue)
  --debug              Enable debug user interface
  --disasm             Enable disassembly output
  --breakpoint <addr>  Set a breakpoint at the specified address (hexadecimal)
  -h, --help           Show this help message
```

## Controls

```
    [ Q ]                    [ O ]
  _/[ E ]\_       ┆┆       _/[ U ]\_
 /         \--------------/         \
|    [W]    |            |    [I]    |
| [A]   [D] |            | [J]   [L] |
|    [S]    | SHFT ENTR⟩ |    [K]    |
|\_________/--------------\_________/|
|       /                    \       |
|      /                      \      |
 \____/                        \____/

  ↑ = W     △ = I     [START]  = Enter
  ← = A     ◻ = J     [SELECT] = R-Shift
  ↓ = S     × = K     [L1] = E  [R1] = U
  → = D     ◯ = L     [L2] = Q  [R2] = O
```

## Dependencies

nuPSX relies on [zig-gamedev](https://github.com/zig-gamedev) libraries for graphics, audio and ImGui bindings.

## Credits

Special thanks to guys in [EmuDev Discord](https://discord.gg/dkmJAes) for sharing tons of undocumented knowledge and getting me unstuck countless times.

**Readings:**
 * [PSX-SPX Playstation Specifications](https://problemkaputt.de/psx-spx.htm) by Martin "nocash" Korth 
 * [PlayStation Emulation](https://jsgroth.dev/blog/posts/ps1-cpu/) blog post series by James Groth
 * [PlayStation Emulation Guide](https://github.com/simias/psx-guide) ([PDF](https://github.com/simias/psx-guide/issues/4#issuecomment-181339467)) by Lionel "simias" Flandrin
 * [The PS1 GPU texture pipeline and how to emulate it](https://www.reddit.com/r/EmuDev/comments/fmhtcn/article_the_ps1_gpu_texture_pipeline_and_how_to/) reddit post by an unknown author

**Tests and Demos:**
 * https://psx.amidog.se/doku.php
 * https://github.com/PeterLemon/PSX
 * https://github.com/JaCzekanski/ps1-tests

**Referenced Projects:**
 * https://github.com/allkern/psxe
 * https://github.com/stenzek/duckstation
 * https://github.com/JaCzekanski/Avocado
