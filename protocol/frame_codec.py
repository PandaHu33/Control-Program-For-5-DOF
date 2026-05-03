import json
import struct
from typing import Dict, Any, List, Tuple

# Simple CRC32 (IEEE) implementation using binascii if available, else manual table
try:
    import binascii
    def crc32(data: bytes) -> int:
        return binascii.crc32(data) & 0xFFFFFFFF
except Exception:
    # Fallback: small table-based implementation
    _CRC32_TABLE = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if (c & 1) else 0)
        _CRC32_TABLE.append(c)
    def crc32(data: bytes) -> int:
        crc = 0xFFFFFFFF
        for b in data:
            crc = _CRC32_TABLE[(crc ^ b) & 0xFF] ^ (crc >> 8)
        return (~crc) & 0xFFFFFFFF


class FrameCodec:
    def __init__(self, schema: Dict[str, Any]):
        self.schema = schema
        self.fields = schema["fields"]
        self.le = schema.get("endianness", "little").lower() == "little"
        self.crc_field = schema.get("crc", {}).get("field", "crc")

    @classmethod
    def from_json_file(cls, path: str):
        with open(path, "r", encoding="utf-8") as f:
            return cls(json.load(f))

    def frame_size(self) -> int:
        total = 0
        for f in self.fields:
            t = f["type"]
            if t == "uint8": total += 1
            elif t == "uint16": total += 2
            elif t == "uint32": total += 4
            elif t == "uint64": total += 8
            elif t == "float32":
                count = f.get("count", 1)
                total += 4 * count
            elif t == "bytes":
                total += int(f["size"])
            elif t == "string":
                total += int(f["size"])  # fixed-size UTF-8, padded/truncated
            else:
                raise ValueError(f"Unknown type: {t}")
        return total

    def pack(self, obj: Dict[str, Any]) -> bytes:
        parts: List[bytes] = []
        for f in self.fields:
            name = f["name"]
            t = f["type"]
            v = obj.get(name)
            endian = "<" if self.le else ">"
            if t == "uint8":
                parts.append(struct.pack(endian + "B", int(v or 0)))
            elif t == "uint16":
                parts.append(struct.pack(endian + "H", int(v or 0)))
            elif t == "uint32":
                parts.append(struct.pack(endian + "I", int(v or 0)))
            elif t == "uint64":
                # Python struct supports Q
                parts.append(struct.pack(endian + "Q", int(v or 0)))
            elif t == "float32":
                count = int(f.get("count", 1))
                arr = list(v or [0.0] * count)
                if len(arr) != count:
                    raise ValueError(f"Field {name} expects {count} items")
                parts.append(struct.pack(endian + ("f" * count), *arr))
            elif t == "bytes":
                size = int(f["size"]) 
                b = bytes(v or b"\x00" * size)
                parts.append(b[:size].ljust(size, b"\x00"))
            elif t == "string":
                size = int(f["size"]) 
                s = str(v or "")
                b = s.encode(f.get("encoding", "utf-8"))[:size]
                parts.append(b.ljust(size, b"\x00"))
            else:
                raise ValueError(f"Unknown type: {t}")
        # compute CRC over all but CRC field
        raw = b"".join(parts)
        # find CRC field range
        # For simplicity, recompute without last 4 bytes if crc at end
        if self.fields[-1]["name"] == self.crc_field and self.fields[-1]["type"] == "uint32":
            payload = raw[:-4]
            val = crc32(payload)
            raw = payload + struct.pack(("<" if self.le else ">") + "I", val)
        return raw

    def unpack(self, data: bytes) -> Dict[str, Any]:
        obj: Dict[str, Any] = {}
        offset = 0
        endian = "<" if self.le else ">"
        for f in self.fields:
            name = f["name"]
            t = f["type"]
            if t == "uint8":
                obj[name] = struct.unpack_from(endian + "B", data, offset)[0]; offset += 1
            elif t == "uint16":
                obj[name] = struct.unpack_from(endian + "H", data, offset)[0]; offset += 2
            elif t == "uint32":
                obj[name] = struct.unpack_from(endian + "I", data, offset)[0]; offset += 4
            elif t == "uint64":
                obj[name] = struct.unpack_from(endian + "Q", data, offset)[0]; offset += 8
            elif t == "float32":
                count = int(f.get("count", 1))
                obj[name] = list(struct.unpack_from(endian + ("f" * count), data, offset))
                offset += 4 * count
            elif t == "bytes":
                size = int(f["size"]) 
                obj[name] = data[offset:offset+size]; offset += size
            elif t == "string":
                size = int(f["size"]) 
                b = data[offset:offset+size]; offset += size
                obj[name] = b.split(b"\x00", 1)[0].decode(f.get("encoding", "utf-8"), errors="ignore")
            else:
                raise ValueError(f"Unknown type: {t}")
        # verify CRC if present at end
        if self.fields[-1]["name"] == self.crc_field and self.fields[-1]["type"] == "uint32":
            expected = obj[self.crc_field]
            calc = crc32(data[:-4])
            obj["crc_ok"] = (calc == expected)
        return obj

if __name__ == "__main__":
    codec = FrameCodec.from_json_file("frame_config.json")
    # derive counts from schema
    def count(name: str, default: int = 1) -> int:
        for f in codec.fields:
            if f.get("name") == name and f.get("type") == "float32":
                return int(f.get("count", default))
        return default
    def size(name: str, default: int = 0) -> int:
        for f in codec.fields:
            if f.get("name") == name and f.get("type") in ("bytes","string"):
                return int(f.get("size", default))
        return default
    sample = {
        "ind": 1,
        "time": 123456789,
        "angle": [0]*count("angle", 7),
        "current": [0]*count("current", 7),
        "torque": [0]*count("torque", 7),
        "pose_ee": [0]*count("pose_ee", 6),
        "pose_elbow": [0]*count("pose_elbow", 6),
        "mode": 1,
        "order": [0.0]*count("order", 16),
        "reserved": b"\x00"*size("reserved", 16),
        "note": "test",
    }
    raw = codec.pack(sample)
    obj = codec.unpack(raw)
    print("size:", len(raw), "crc_ok:", obj.get("crc_ok"))
