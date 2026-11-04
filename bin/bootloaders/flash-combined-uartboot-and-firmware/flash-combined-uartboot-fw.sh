#!/bin/sh

set -eu

repo="openenergymonitor/emon32-fw"
output="emon32-latest.elf"
temporary_output="${output}.tmp"

elf_url=$(
    curl -fsSL "https://api.github.com/repos/${repo}/releases/latest" |
        awk '
            /"browser_download_url"[[:space:]]*:[[:space:]]*"[^"]*\/emon32-v[^"]*\.elf"/ {
                url = $0
                sub(/^.*"browser_download_url"[[:space:]]*:[[:space:]]*"/, "", url)
                sub(/".*$/, "", url)
                matches++
                match_url = url
            }
            END {
                if (matches != 1) {
                    print "expected exactly one emon32 firmware ELF asset" > "/dev/stderr"
                    exit 1
                }
                print match_url
            }
        '
)

trap 'rm -f "$temporary_output"' EXIT HUP INT TERM
curl -fL --retry 3 --output "$temporary_output" "$elf_url"
mv "$temporary_output" "$output"
trap - EXIT HUP INT TERM

arm-none-eabi-gdb -nx --batch  -x hex_bmp_flash.gdb
arm-none-eabi-gdb -nx --batch  -x elf_bmp_flash.gdb

echo "\n\n---- DONE! ----"
