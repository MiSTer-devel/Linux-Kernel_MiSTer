#!/usr/bin/env bash
#
# build-mister-modules.sh — build the out-of-tree drivers this tree vendors.
#
# The kernel builds with:
#     make ARCH=arm MiSTer_defconfig
#     make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- zImage
#
# These drivers do NOT build from that, by design. They are vendored at the paths the
# 5.15 branch uses, but they are not wired into Kconfig, because their own Makefiles do
# parse-time work keyed off `$(shell pwd)`:
#
#     export TopDIR ?= $(shell pwd)
#     $(shell cp $(TopDIR)/autoconf_..._linux.h $(TopDIR)/include/autoconf.h)
#
# In an in-tree build `pwd` is the kernel root rather than the module directory, so that
# copy silently lands in the wrong place and the driver's generated autoconf.h never
# appears. These Makefiles assume they are always built out-of-tree. So that is how this
# builds them — which is upstream's own supported path, not a workaround.
#
# Usage: ./build-mister-modules.sh [ARCH] [CROSS_COMPILE]
#   defaults: arm, arm-linux-gnueabihf-
#
# REQUIRES A FULLY BUILT KERNEL FIRST -- not just `modules_prepare`:
#
#     make ARCH=arm MiSTer_defconfig
#     make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- LOCALVERSION= zImage modules
#     ./build-mister-modules.sh
#
# PASS `LOCALVERSION=` TO THE KERNEL BUILD, exactly as above. Empty, but SET.
#
# This tree is a git repo whose HEAD sits dozens of commits past the v<ver> base commit, so
# scripts/setlocalversion correctly concludes the source is modified and appends a "+",
# giving `6.18.38+`. Buildroot builds the same source from a tarball with no git around
# it, so its identical-but-also-patched kernel reports plain `6.18.38`. The two disagree
# only because one build can see its own history and the other cannot.
#
# That single "+" lands in vermagic, and vermagic is what the kernel matches on when
# loading a module:
#
#     vermagic=6.18.38+ SMP mod_unload ARMv7 p2v8     <- built here, without LOCALVERSION=
#     vermagic=6.18.38  SMP mod_unload ARMv7 p2v8     <- Buildroot, and this tree WITH it
#
# Mismatch that and modprobe rejects every module ("version magic ... should be ..."),
# which reads like a broken driver and is not. Setting LOCALVERSION= (even to empty)
# makes setlocalversion skip the "+" entirely, so this tree's kernel and modules are
# interchangeable with the shipped image's.
#
# `modules_prepare` is NOT enough, and the way it fails is worth knowing because the
# error blames the driver rather than the real cause. An external module is linked
# against the kernel's symbol table in Module.symvers, and that file is produced by
# modpost during `make modules`, which in turn needs vmlinux from the `zImage` build.
# Without it every kernel symbol the driver uses reads as undefined:
#
#     ERROR: modpost: "skb_pull" [8812au.ko] undefined!
#
# Nothing is wrong with the driver there -- the kernel symbol table simply is not built
# yet. Hence the check below, which says so directly.

set -o errexit
set -o nounset
set -o pipefail

readonly KDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly ARCH="${1:-arm}"
readonly CROSS_COMPILE="${2:-arm-linux-gnueabihf-}"

[[ -f $KDIR/.config ]] || {
	printf 'No .config — run `make ARCH=%s MiSTer_defconfig` first.\n' "$ARCH" >&2
	exit 1
}

[[ -f $KDIR/Module.symvers ]] || {
	cat >&2 <<EOF
No Module.symvers — the kernel is not built yet, so modpost has no symbol table and
every kernel symbol these drivers use would be reported as undefined.

Build the kernel first, then re-run this:

    make ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE zImage modules

(\`modules_prepare\` alone does NOT produce Module.symvers -- it needs vmlinux.)
EOF
	exit 1
}

build_module() {
	local dir="$1"; shift
	printf '\n=== %s\n' "$dir"
	# Exactly what Buildroot's kernel-module infra invokes, including each driver's own
	# CONFIG_ override where its Makefile gates obj- behind one.
	#
	# LOCALVERSION= (set, but empty) is load-bearing -- see the note at the top of this
	# script. It must match the kernel build's, or these modules get a vermagic the
	# kernel rejects.
	make -C "$KDIR" ARCH="$ARCH" CROSS_COMPILE="$CROSS_COMPILE" LOCALVERSION= \
		M="$dir" "$@" modules
}

build_module drivers/net/wireless/realtek/rtl8812au CONFIG_RTL8812AU=m
build_module drivers/net/wireless/realtek/rtl8821au CONFIG_RTL8821AU=m
build_module drivers/hid/xone

printf '\nBuilt .ko files:\n'
find . -name '*.ko' -newer .config -printf '  %p\n' 2>/dev/null | sort
