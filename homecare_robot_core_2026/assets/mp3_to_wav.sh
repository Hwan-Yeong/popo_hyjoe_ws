#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'USAGE'
Usage:
  mp3_to_wav.sh input.mp3 [output.wav]
  mp3_to_wav.sh -d INPUT_DIR [-o OUTPUT_DIR] [-r 48000] [-c 2]

Options:
  -d DIR   Convert all .mp3 files in DIR (non-recursive)
  -o DIR   Output directory for batch mode (default: same as input dir)
  -r RATE  Sample rate in Hz (default: 48000)
  -c CH    Channels: 1=mono, 2=stereo (default: 2)
  -h       Show this help

Examples:
  mp3_to_wav.sh bgm.mp3
  mp3_to_wav.sh bgm.mp3 bgm.wav
  mp3_to_wav.sh -d ./music
  mp3_to_wav.sh -d ./music -o ./wav_out -r 44100 -c 1
USAGE
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "[ERROR] '$1' is not installed." >&2
    exit 1
  }
}

convert_one() {
  local in_file="$1"
  local out_file="$2"
  local rate="$3"
  local channels="$4"

  if [[ ! -f "$in_file" ]]; then
    echo "[ERROR] Input file not found: $in_file" >&2
    return 1
  fi

  mkdir -p "$(dirname "$out_file")"

  echo "[INFO] Converting: $in_file -> $out_file"
  ffmpeg -y -hide_banner -loglevel error \
    -i "$in_file" \
    -ar "$rate" \
    -ac "$channels" \
    -c:a pcm_s16le \
    "$out_file"

  echo "[OK] Created: $out_file"
}

require_cmd ffmpeg

sample_rate=48000
channels=2
batch_mode=0
input_dir=""
output_dir=""

while getopts ":d:o:r:c:h" opt; do
  case "$opt" in
    d) batch_mode=1; input_dir="$OPTARG" ;;
    o) output_dir="$OPTARG" ;;
    r) sample_rate="$OPTARG" ;;
    c) channels="$OPTARG" ;;
    h) usage; exit 0 ;;
    \?) echo "[ERROR] Invalid option: -$OPTARG" >&2; usage; exit 1 ;;
    :) echo "[ERROR] Option -$OPTARG requires an argument." >&2; usage; exit 1 ;;
  esac
done
shift $((OPTIND - 1))

if [[ "$channels" != "1" && "$channels" != "2" ]]; then
  echo "[ERROR] Channels must be 1 or 2." >&2
  exit 1
fi

if [[ "$batch_mode" -eq 1 ]]; then
  if [[ -z "$input_dir" || ! -d "$input_dir" ]]; then
    echo "[ERROR] Valid input directory is required for batch mode." >&2
    exit 1
  fi

  if [[ -z "$output_dir" ]]; then
    output_dir="$input_dir"
  fi
  mkdir -p "$output_dir"

  shopt -s nullglob
  files=("$input_dir"/*.mp3 "$input_dir"/*.MP3)
  shopt -u nullglob

  if [[ ${#files[@]} -eq 0 ]]; then
    echo "[ERROR] No MP3 files found in: $input_dir" >&2
    exit 1
  fi

  for src in "${files[@]}"; do
    base="$(basename "$src")"
    stem="${base%.*}"
    dst="$output_dir/$stem.wav"
    convert_one "$src" "$dst" "$sample_rate" "$channels"
  done
  exit 0
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 1
fi

input_file="$1"
output_file="${2:-${input_file%.*}.wav}"
convert_one "$input_file" "$output_file" "$sample_rate" "$channels"

