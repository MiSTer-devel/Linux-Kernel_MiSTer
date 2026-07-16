# This tree is generated

It is a **build output**, not a source of truth. It was rendered from
[Buildroot_MiSTer](https://github.com/mcfbytes/Buildroot_MiSTer) by
`scripts/export-kernel-tree.sh`, which is where the kernel is actually maintained.

**Changes made directly to this tree will be erased by the next regeneration.**
To change the kernel, change the patch series in Buildroot_MiSTer and regenerate. There
are two series, and which one a patch belongs in is the first question to answer:

- `board/mister/de10nano/linux-patches/` — **carried**. Applied by Buildroot to the
  shipped MiSTer image *and* replayed here. Anything the image needs goes here.
- `board/mister/de10nano/linux-patches-upstream/` — **upstream-only**. Replayed here
  and nowhere else; Buildroot never reads this directory. Numbered from 0100 so the
  namespace is obvious from a filename alone.

## This tree is NOT the kernel the MiSTer image ships

It is that kernel **plus exactly 1 patch**, listed here. Every one is
carried for *this* tree only: Buildroot's `BR2_LINUX_KERNEL_PATCH` points at
`board/mister/de10nano/linux-patches/` and never sees them, so they are in no MiSTer
image this repo builds. They are here because Linux-Kernel_MiSTer is upstream's kernel
for every MiSTer, not only ours, and it has to keep working for all of them.

| patch | not in the MiSTer image because |
|---|---|
| init: support for init loop device | Buildroot_MiSTer boots through an initramfs /init that performs these mounts from userspace, so the in-kernel loop= path would be unreachable code in the image it builds; every stock MiSTer boots through it, so the exported tree has to keep it working. |

They are the last 1 commit of the patch series here, immediately after
the carried ones, so the split is visible in the log as well as in this table:

    git log --oneline mister-6.18.38~7..mister-6.18.38~6

## What is here

| | |
|---|---|
| Base | Pristine Linux 6.18.38 from kernel.org, hash-verified (``) |
| Carried patches | 31 commits, one per patch the MiSTer image applies, original authorship preserved |
| Upstream-only patches | 1 commit, one per patch carried for this tree alone (see above) |
| Config | `arch/arm/configs/MiSTer_defconfig` — copied verbatim from linux.config |
| Vendored drivers | 3 commits, one per out-of-tree kernel module (see below) |
| Tag | `mister-6.18.38` |

The base commit contains no MiSTer change, so the two deltas worth looking at are:

    git diff mister-6.18.38~38 mister-6.18.38              # everything MiSTer adds to upstream 6.18.38
    git diff mister-6.18.38~38 mister-6.18.38~7   # only what the MiSTer image ships

The first is this tree in full: both patch series, the in-tree defconfig, and the
vendored out-of-tree drivers. The second stops at the last carried patch, so it is
precisely the patch set Buildroot applies when it builds the image — no upstream-only
patches, and none of the packaging commits that Buildroot supplies from its own tree
rather than from the kernel source.

## Building standalone

    make ARCH=arm MiSTer_defconfig
    make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOCALVERSION= zImage modules
    ./build-mister-modules.sh

Three details on that middle line, each of which will bite you otherwise.

**`LOCALVERSION=`** — empty, but set. This tree is a git repo whose HEAD is
38 commits past the `v6.18.38` base, so `scripts/setlocalversion` correctly
calls the source modified and appends `+`, giving `6.18.38+`. Buildroot patches a
tarball with no git around it, so the kernel it builds reports plain `6.18.38`. That
`+` lands in **vermagic**, and vermagic is what the kernel matches on when loading
modules:

    vermagic=6.18.38+ SMP mod_unload ARMv7 p2v8     <- without LOCALVERSION=
    vermagic=6.18.38  SMP mod_unload ARMv7 p2v8     <- Buildroot, and here WITH it

Mismatch it and modprobe rejects every module. With it, modules built here load into the
shipped image's kernel and vice versa. Note that this makes the two **module-compatible**,
not identical: the kernel image built here is not byte-identical to the shipped one
whenever the upstream-only series above is non-empty, because the shipped one does not
contain those patches.

**`modules`** — not just `zImage`. External modules link against the kernel symbol
table in `Module.symvers`, which modpost writes during `make modules` (and which needs
`vmlinux` first). `modules_prepare` does **not** produce it, and without it modpost
calls every kernel symbol undefined
(`ERROR: modpost: "skb_pull" [8812au.ko] undefined!`) — which looks like a broken driver
and is not. `build-mister-modules.sh` checks for this and says so.

**The third line at all** — the Xbox (xone) and 11ac WiFi drivers are out-of-tree, so
`zImage` never builds them.

Building the kernel also needs `lz4` on the host, since this config sets
`CONFIG_KERNEL_LZ4`.

## Vendored out-of-tree drivers

| path | pin | build override |
|---|---|---|
| `drivers/net/wireless/realtek/rtl8812au` | 8cac6f43316a56cc89cc8cb532cd6c6ae14c4805 | CONFIG_RTL8812AU=m |
| `drivers/net/wireless/realtek/rtl8821au` | 3a7cdb591b64d99d2670e455bde67c8ab338525b | CONFIG_RTL8821AU=m |
| `drivers/hid/xone` | f2aa9fe01103d7600553b505b298ff0bd47ff280 | — |

Sources are verbatim upstream at the paths `MiSTer-v5.15` uses, so the layout matches.
They are deliberately **not** wired into Kconfig. Their own Makefiles do parse-time work
keyed off `$(shell pwd)`:

    export TopDIR ?= $(shell pwd)
    $(shell cp $(TopDIR)/autoconf_..._linux.h $(TopDIR)/include/autoconf.h)

In an in-tree build `pwd` is the kernel root rather than the module directory, so that
copy lands in the wrong place and the driver's generated `autoconf.h` never appears —
silently, because `$(shell ...)` swallows the error. These Makefiles assume they are
always built out-of-tree. `build-mister-modules.sh` therefore uses upstream's own
supported out-of-tree path, which is also exactly what Buildroot invokes to build the
shipped image — so it is a proven recipe rather than a workaround.

The pins live in `package/<name>/<name>.mk` in Buildroot_MiSTer. Bumping a driver is a
pin change there plus a re-run of the export; nothing here needs rewiring.

Unlike `MiSTer-v5.15`, which vendors these in-tree, that means a driver bump does not
touch this tree's history by hand — and the Realtek drivers here track upstreams that
build against 6.18 with **zero** compatibility patches.

## Where this branch hangs

This repo's tarball commits form a spine, and each `MiSTer-vX.Y` branch hangs off a
spine point with the MiSTer series replayed on top. This branch extends that spine the
same way, so it is the next entry rather than a foreign import:

    e12ed6c19 v5.13.12 -> 137491a75 v5.14 -> b6f2ca1c4 v5.14.5 -> aba1ef4c1 v5.15.1
                                                                       |
                                          +----------------------------+
                                          |
       [112 MiSTer commits] -> MiSTer-v5.15        (untouched)
                                          |
       v6.18.38 -> [31 carried + 1 upstream-only] -> MiSTer-v6.18

`MiSTer-v5.15` is **not modified and not an ancestor** — it is a sibling, exactly as
`MiSTer-v5.14` already is. Nothing was lost.

Two consequences worth knowing:

- The base commit's parent is itself a pristine tarball commit, so
  `git diff aba1ef4c1 v6.18.38` is the **pure upstream 5.15.1 → 6.18.38 delta**,
  with no MiSTer code on either side.
- No MiSTer-5.15 commit appears in this branch's log, which is the point: this tree
  does not contain most of them, and a log listing changes that are absent from the
  tree would be worse than no log at all.

What each 5.15 commit became — carried into the image, carried here only (the
upstream-only series above), superseded by an upstream commit (with the vanilla commit
cited), or deliberately dropped — is recorded per commit in
`MISTER-KERNEL-PATCH-RECON.md` in Buildroot_MiSTer. No git command can answer that:
across this much context drift `git patch-id` matches nothing, so "is this commit in
6.18.38?" is a semantic question, not a mechanical one.

## Publishing

This script never touches a remote. To publish, fetch the orphan branch into a fork
and push from there:

    git -C <your-fork> fetch <this-export-dir> MiSTer-v6.18:MiSTer-v6.18
    git -C <your-fork> push origin MiSTer-v6.18

