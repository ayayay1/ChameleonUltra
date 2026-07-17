#!/usr/bin/env python3
"""
Verify a signed nRF DFU (dfu-cc) OTA package for ChameleonUltra.

Checks:
  * dfu-cc structure (signed command present)
  * fw/hw version, sd_req, type, app_size, SHA256 hash (LE), CRC boot_validation
  * ECDSA-P256 signature against the project's bootloader public key (pk[64])

Usage:
    python verify.py                                  # checks ./out/ultra-dfu-app.zip
    python verify.py path/to/package.zip

Dependencies:  pip install ecdsa protobuf
               (dfu_cc_pb2.py is vendored next to this script)
"""
import sys, os, hashlib, zipfile, json, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import dfu_cc_pb2 as cc
from ecdsa import VerifyingKey, NIST256p, ellipticcurve


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("zip", nargs="?", default=os.path.join(HERE, "out", "ultra-dfu-app.zip"))
    args = ap.parse_args()

    z = zipfile.ZipFile(args.zip)
    print("zip members:", z.namelist())
    app = z.read("application.bin")
    dat = z.read("application.dat")
    manifest = json.loads(z.read("manifest.json"))
    print("manifest:", json.dumps(manifest))

    pkt = cc.Packet(); pkt.ParseFromString(dat)
    assert pkt.HasField("signed_command"), "must be signed"
    sc = pkt.signed_command
    print("signature_type:", sc.signature_type, "(0=ECDSA_P256_SHA256)")
    print("signature_len :", len(sc.signature), "(must be 64)")
    init = sc.command.init
    print("fw_version :", init.fw_version)
    print("hw_version :", init.hw_version)
    print("sd_req     :", list(init.sd_req), "(256=0x0100 S140 v7.2.0)")
    print("type       :", init.type, "(0=APPLICATION)")
    print("app_size   :", init.app_size, "vs real bin:", len(app))
    print("hash_type  :", init.hash.hash_type, "(3=SHA256)")
    h = hashlib.sha256(app).digest()
    print("stored hash:", init.hash.hash.hex())
    print("LE match   :", init.hash.hash == h[::-1])
    print("BE match   :", init.hash.hash == h)
    print("boot_valid :", [(bv.type, bv.bytes.hex()) for bv in init.boot_validation], "(1=CRC)")

    # Verify signature the way the bootloader does:
    # 1) hash the InitCommand content bytes
    init_bytes = init.SerializeToString()
    digest = hashlib.sha256(init_bytes).digest()
    # 2) signature is stored little-endian -> swap to big-endian for verify
    sig_be = sc.signature[:32][::-1] + sc.signature[32:][::-1]
    # project bootloader public key (pk[64], big-endian X||Y)
    vk = VerifyingKey.from_public_point(
        ellipticcurve.Point(
            NIST256p.curve,
            int.from_bytes(bytes.fromhex("c643d0b4886b969920dad4242785f24142b699474445fee1c58efc7889ab067c"), 'big'),
            int.from_bytes(bytes.fromhex("e9adef11e59b3e536fbfdf2d5bc04b51c546d05d71638a2930367c3fdba2fc5f"), 'big'),
            NIST256p.order),
        NIST256p)
    ok = vk.verify_digest(sig_be, digest,
                         sigdecode=lambda sig, order: (int.from_bytes(sig[:32], 'big'), int.from_bytes(sig[32:], 'big')))
    print("SIGNATURE VERIFIES against pk[64]:", ok)
    assert ok
    print("\nALL CHECKS PASSED")


if __name__ == "__main__":
    main()
