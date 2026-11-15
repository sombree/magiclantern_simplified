#!/usr/bin/env python3
"""Extract EOS RP native-CRX frames from MLV.

Supported input layouts:
  - prototype/legacy: CRXI v1 + CRXF
  - recorder revision 2: CRXI v2 + standard VIDF

Revision 2 stores one exact Canon CMP1 in CRXI and each VIDF.frameData is the
exact Canon CRX sample.  No CRXF-specific frame bookkeeping is needed.
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path

MLVI = int.from_bytes(b"MLVI", "little")
CRXI = int.from_bytes(b"CRXI", "little")
CRXF = int.from_bytes(b"CRXF", "little")
VIDF = int.from_bytes(b"VIDF", "little")

CRXI_V1 = struct.Struct("<IIQIIIIHHHHIIII")
CRXI_V2 = struct.Struct("<IIQIIIiII")
CRXF_FIXED = struct.Struct("<IIQIHHHH" + "I" * 16)
SEG_FMT = struct.Struct("<III")
VIDF_FIXED = struct.Struct("<IIQIHHHHI")

CRX_VIDEO_CLASS_FLAG = 0x0200
RAW_VIDEO_CLASS = 0x0001


def fourcc(v: int) -> str:
    return int(v).to_bytes(4, "little", signed=False).decode("ascii", errors="replace")


def be16(b: bytes, off: int) -> int:
    return int.from_bytes(b[off:off + 2], "big")


def be32(b: bytes, off: int) -> int:
    return int.from_bytes(b[off:off + 4], "big")


def parse_cmp1(cmp1: bytes) -> dict:
    if len(cmp1) < 32:
        raise ValueError(f"CMP1 too small: {len(cmp1)}")
    return {
        "size": len(cmp1),
        "version": be16(cmp1, 4),
        "f_width": be32(cmp1, 8),
        "f_height": be32(cmp1, 12),
        "tile_width": be32(cmp1, 16),
        "tile_height": be32(cmp1, 20),
        "n_bits": cmp1[24],
        "n_planes": cmp1[25] >> 4,
        "cfa_layout": cmp1[25] & 0x0F,
        "enc_type": cmp1[26] >> 4,
        "image_levels": cmp1[26] & 0x0F,
        "mdat_hdr_size": be32(cmp1, 28),
    }


def scan_blocks(data: bytes):
    off = 0
    while off + 8 <= len(data):
        block_type, block_size = struct.unpack_from("<II", data, off)
        if block_size < 8 or off + block_size > len(data):
            raise ValueError(
                f"invalid MLV block at 0x{off:X}: type={fourcc(block_type)!r} size={block_size}"
            )
        yield off, block_type, block_size
        off += block_size
    if off != len(data):
        raise ValueError(f"trailing {len(data) - off} bytes after last complete MLV block")


def parse_mlvi(data: bytes, off: int, size: int) -> dict:
    if size < 0x34:
        raise ValueError(f"MLVI too small at 0x{off:X}: {size}")
    video_class = struct.unpack_from("<H", data, off + 0x20)[0]
    video_frames = struct.unpack_from("<I", data, off + 0x24)[0]
    return {
        "offset": off,
        "size": size,
        "video_class": video_class,
        "video_frame_count": video_frames,
        "raw": bool(video_class & 0x000F == RAW_VIDEO_CLASS),
        "crx": bool(video_class & CRX_VIDEO_CLASS_FLAG),
    }


def parse_crxi(data: bytes, off: int, size: int) -> dict:
    if size < 20:
        raise ValueError(f"CRXI at 0x{off:X} too small: {size}")
    version = struct.unpack_from("<I", data, off + 16)[0]

    if version == 1:
        if size < CRXI_V1.size:
            raise ValueError(f"CRXI v1 at 0x{off:X} too small: {size}")
        v = CRXI_V1.unpack_from(data, off)
        return {
            "offset": off,
            "size": v[1],
            "timestamp": v[2],
            "version": v[3],
            "flags": v[4],
            "frame_block_type": fourcc(v[5]),
            "descriptor_size": v[6],
            "res_x": v[7],
            "res_y": v[8],
            "bits_per_pixel": v[9],
            "profile": v[10],
            "max_segments": v[11],
            "cmp1": None,
        }

    if version == 2:
        if size < CRXI_V2.size:
            raise ValueError(f"CRXI v2 at 0x{off:X} too small: {size}")
        v = CRXI_V2.unpack_from(data, off)
        header_size = v[4]
        flags = v[5]
        value_shift = v[6]
        cmp1_size = v[7]
        reserved = v[8]
        if header_size < CRXI_V2.size:
            raise ValueError(f"CRXI v2 headerSize {header_size} < 40")
        if header_size > size or cmp1_size > size - header_size:
            raise ValueError("CRXI v2 CMP1 outside block")
        if size != header_size + cmp1_size:
            raise ValueError(
                f"CRXI v2 must be exact: blockSize={size}, headerSize={header_size}, cmp1Size={cmp1_size}"
            )
        if flags != 0 or reserved != 0:
            raise ValueError(f"CRXI v2 reserved fields nonzero flags={flags:#x} reserved={reserved:#x}")
        cmp1 = data[off + header_size: off + header_size + cmp1_size]
        return {
            "offset": off,
            "size": v[1],
            "timestamp": v[2],
            "version": v[3],
            "header_size": header_size,
            "flags": flags,
            "value_shift": value_shift,
            "cmp1_size": cmp1_size,
            "reserved": reserved,
            "cmp1_info": parse_cmp1(cmp1),
            "cmp1": cmp1,
        }

    raise ValueError(f"unsupported CRXI version {version} at 0x{off:X}")


def parse_vidf(data: bytes, off: int, size: int) -> dict:
    if size < VIDF_FIXED.size:
        raise ValueError(f"VIDF at 0x{off:X} too small: {size}")
    v = VIDF_FIXED.unpack_from(data, off)
    frame_space = v[8]
    if frame_space > size - VIDF_FIXED.size:
        raise ValueError(f"VIDF frameSpace outside block: {frame_space}")
    payload_off = off + VIDF_FIXED.size + frame_space
    payload_size = size - VIDF_FIXED.size - frame_space
    if payload_size == 0:
        raise ValueError("CRX VIDF has empty payload")
    return {
        "offset": off,
        "size": size,
        "timestamp": v[2],
        "frame_number": v[3],
        "crop_pos_x": v[4],
        "crop_pos_y": v[5],
        "pan_pos_x": v[6],
        "pan_pos_y": v[7],
        "frame_space": frame_space,
        "sample_size": payload_size,
        "sample": data[payload_off:off + size],
    }


def parse_crxf(data: bytes, off: int, size: int) -> dict:
    """Legacy prototype CRXF adapter."""
    if size < CRXF_FIXED.size:
        raise ValueError(f"CRXF at 0x{off:X} too small: {size}")
    v = CRXF_FIXED.unpack_from(data, off)
    meta = {
        "offset": off,
        "size": v[1],
        "timestamp": v[2],
        "frame_number": v[3],
        "width": v[4],
        "height": v[5],
        "bits_per_pixel": v[6],
        "profile": v[7],
        "header_bytes": v[8],
        "main_header_offset": v[9],
        "main_header_size": v[10],
        "subheader_offset": v[11],
        "subheader_size": v[12],
        "ring_slots": v[13],
        "segment_count": v[14],
        "logical_sample_size": v[15],
        "input_copy_us": v[16],
        "encode_us": v[17],
        "detach_us": v[18],
        "submit_rc": v[19],
        "release_rc": v[20],
        "m2m_blocks": v[21],
        "m2m_bytes": v[22],
        "flags": v[23],
    }

    seg_count = meta["segment_count"]
    table_end = CRXF_FIXED.size + seg_count * SEG_FMT.size
    if seg_count > 64 or meta["header_bytes"] < table_end or meta["header_bytes"] > size:
        raise ValueError(f"CRXF frame {meta['frame_number']}: invalid segment table")

    def region(rel: int, n: int, label: str) -> bytes:
        if rel > size or n > size - rel:
            raise ValueError(
                f"CRXF frame {meta['frame_number']}: {label} outside block rel=0x{rel:X} size={n}"
            )
        return data[off + rel: off + rel + n]

    segments = []
    for i in range(seg_count):
        ring_index, rel, n = SEG_FMT.unpack_from(
            data, off + CRXF_FIXED.size + i * SEG_FMT.size
        )
        segments.append({
            "ring_index": ring_index,
            "offset": rel,
            "size": n,
            "payload": region(rel, n, f"segment[{i}]"),
        })

    main = region(meta["main_header_offset"], meta["main_header_size"], "main header")
    sub = region(meta["subheader_offset"], meta["subheader_size"], "subheader")
    segments.sort(key=lambda x: x["ring_index"])
    sample = sub + b"".join(x["payload"] for x in segments)
    if len(sample) != meta["logical_sample_size"]:
        raise ValueError(
            f"CRXF frame {meta['frame_number']}: logical sample {len(sample)} != declared {meta['logical_sample_size']}"
        )

    meta["segments"] = [{k: s[k] for k in ("ring_index", "offset", "size")} for s in segments]
    meta["main_header"] = main
    meta["sample"] = sample
    return meta


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mlv", type=Path)
    ap.add_argument("-o", "--output", type=Path, default=None)
    args = ap.parse_args()

    data = args.mlv.read_bytes()
    outdir = args.output or args.mlv.with_name(args.mlv.stem + "_crx")
    outdir.mkdir(parents=True, exist_ok=True)

    mlvi = None
    stream_info = []
    active_crxi = None
    frame_meta = []

    for off, typ, size in scan_blocks(data):
        if typ == MLVI:
            mlvi = parse_mlvi(data, off, size)
        elif typ == CRXI:
            info = parse_crxi(data, off, size)
            stream_info.append({k: v for k, v in info.items() if k != "cmp1"})
            active_crxi = info
            if info["version"] == 2:
                cmp1_path = outdir / "stream.cmp1"
                cmp1_path.write_bytes(info["cmp1"])
                ci = info["cmp1_info"]
                print(
                    f"CRXI v2 @0x{off:X}: CMP1={info['cmp1_size']} "
                    f"{ci['f_width']}x{ci['f_height']} {ci['n_bits']}b "
                    f"enc={ci['enc_type']} levels={ci['image_levels']} "
                    f"shift={info['value_shift']}"
                )
            else:
                print(
                    f"CRXI v1 @0x{off:X}: {info['res_x']}x{info['res_y']} "
                    f"{info['bits_per_pixel']}b profile={info['profile']}"
                )
        elif typ == CRXF:
            frame = parse_crxf(data, off, size)
            n = frame["frame_number"]
            sample_path = outdir / f"frame_{n:06d}.crxsample"
            main_path = outdir / f"frame_{n:06d}.cmp1"
            sample_path.write_bytes(frame.pop("sample"))
            main_path.write_bytes(frame.pop("main_header"))
            frame["container"] = "CRXF-v1"
            frame_meta.append(frame)
            print(
                f"CRXF {n:6d}: sample={frame['logical_sample_size']} "
                f"seg={frame['segment_count']} enc={frame['encode_us']}us"
            )
        elif typ == VIDF and active_crxi and active_crxi["version"] == 2:
            frame = parse_vidf(data, off, size)
            n = frame["frame_number"]
            sample_path = outdir / f"frame_{n:06d}.crxsample"
            sample_path.write_bytes(frame.pop("sample"))
            frame["container"] = "VIDF-rev2"
            frame_meta.append(frame)
            print(
                f"VIDF {n:6d}: sample={frame['sample_size']} "
                f"frameSpace={frame['frame_space']} block={frame['size']}"
            )

    if mlvi and mlvi["crx"] and not stream_info:
        raise ValueError("MLVI advertises CRX but no CRXI was found")

    if active_crxi and active_crxi["version"] == 2:
        if any(f["container"] != "VIDF-rev2" for f in frame_meta):
            raise ValueError("revision-2 stream contains non-VIDF CRX frames")

    summary = {
        "source": str(args.mlv),
        "mlvi": mlvi,
        "crxi": stream_info,
        "frames": frame_meta,
        "frame_count": len(frame_meta),
    }
    (outdir / "metadata.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(f"Extracted {len(frame_meta)} CRX frame(s) to {outdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
