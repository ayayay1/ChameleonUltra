#!/usr/bin/env python3
"""
Build a signed nRF DFU (dfu-cc) OTA package for ChameleonUltra.

Produces application.bin / application.dat / manifest.json and a
`<name>.zip` that can be flashed with any DFU-capable GUI/CLI.

Usage:
    python generate.py                      # repo defaults, output -> ./out
    python generate.py --out out            # output directory (default: ./out)
    python generate.py --app-bin path/to/application.bin --pem path/to/chameleon.pem

The signing key (resource/dfu_key/chameleon.pem) is the project's public
DFU development key; the matching public point is baked into the bootloader,
so the resulting package validates on stock ChameleonUltra bootloaders.

Dependencies:  pip install ecdsa protobuf
               (dfu_cc_pb2.py is vendored next to this script)
"""
import sys, hashlib, zipfile, os, json, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
# repo root is the parent of tools/ota/ (i.e. two levels up from this file)
REPO_DEFAULT = os.path.dirname(os.path.dirname(HERE))

# import the vendored protobuf schema (same directory)
sys.path.insert(0, HERE)
import dfu_cc_pb2 as cc
from ecdsa import SigningKey


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=REPO_DEFAULT, help="ChameleonUltra repo root")
    ap.add_argument("--app-bin", default=None, help="path to application.bin")
    ap.add_argument("--pem", default=None, help="path to resource/dfu_key/chameleon.pem")
    ap.add_argument("--out", default=os.path.join(HERE, "out"), help="output directory")
    ap.add_argument("--name", default="ultra-dfu-app", help="output zip base name")
    args = ap.parse_args()

    repo = args.repo
    app_bin = args.app_bin or os.path.join(repo, "firmware", "objects", "application.bin")
    pem = args.pem or os.path.join(repo, "resource", "dfu_key", "chameleon.pem")
    out_dir = args.out

    if not os.path.exists(app_bin):
        sys.exit("ERROR: application.bin not found at %s (build the firmware first)" % app_bin)
    if not os.path.exists(pem):
        sys.exit("ERROR: signing key not found at %s" % pem)

    app = open(app_bin, "rb").read()
    app_size = len(app)
    app_hash = hashlib.sha256(app).digest()
    app_hash_le = app_hash[::-1]  # bootloader expects little-endian

    # ---- Build InitCommand (matches the proven ZhaiRen package) ----
    init = cc.InitCommand()
    init.fw_version = 1
    init.hw_version = 0
    init.sd_req.append(256)          # 0x0100 = S140 v7.2.0 FWID
    init.type = 0                    # APPLICATION
    init.app_size = app_size
    init.hash.hash_type = 3          # SHA256
    init.hash.hash = app_hash_le
    init.is_debug = False
    bv = init.boot_validation.add()
    bv.type = 1                      # VALIDATE_CRC (bootloader recomputes CRC itself)
    bv.bytes = b''

    init_bytes = init.SerializeToString()
    print("init_bytes len:", len(init_bytes))

    # ---- Sign SHA256(InitCommand) with chameleon.pem (ECDSA-P256) ----
    sk = SigningKey.from_pem(open(pem, "rb").read())
    digest = hashlib.sha256(init_bytes).digest()
    sig_be = sk.sign_digest(digest, sigencode=lambda r, s, order: r.to_bytes(32, 'big') + s.to_bytes(32, 'big'))
    sig_le = sig_be[:32][::-1] + sig_be[32:][::-1]   # bootloader stores signature little-endian

    # ---- Assemble Packet.signed_command ----
    cmd = cc.Command()
    cmd.op_code = 1                  # INIT
    cmd.init.CopyFrom(init)
    sc = cc.SignedCommand()
    sc.command.CopyFrom(cmd)
    sc.signature_type = 0            # ECDSA_P256_SHA256
    sc.signature = sig_le
    pkt = cc.Packet()
    pkt.signed_command.CopyFrom(sc)
    dat = pkt.SerializeToString()

    # ---- Self-verification (mimic bootloader) ----
    pkt2 = cc.Packet(); pkt2.ParseFromString(dat)
    assert pkt2.HasField("signed_command")
    reinit = pkt2.signed_command.command.init.SerializeToString()
    assert reinit == init_bytes, "re-serialized init mismatch!"
    vk = sk.get_verifying_key()
    sig_be_check = sig_le[:32][::-1] + sig_le[32:][::-1]
    verified = vk.verify_digest(sig_be_check, digest,
                                sigdecode=lambda sig, order: (int.from_bytes(sig[:32], 'big'), int.from_bytes(sig[32:], 'big')))
    print("signature verifies against init bytes:", verified)
    assert verified

    # ---- Write artifacts ----
    os.makedirs(out_dir, exist_ok=True)
    open(os.path.join(out_dir, "application.bin"), "wb").write(app)
    open(os.path.join(out_dir, "application.dat"), "wb").write(dat)
    manifest = {"manifest": {"application": {"bin_file": "application.bin", "dat_file": "application.dat"}}}
    open(os.path.join(out_dir, "manifest.json"), "w").write(json.dumps(manifest, indent=4))

    zip_path = os.path.join(out_dir, args.name + ".zip")
    if os.path.exists(zip_path):
        os.remove(zip_path)
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(os.path.join(out_dir, "application.bin"), "application.bin")
        z.write(os.path.join(out_dir, "application.dat"), "application.dat")
        z.write(os.path.join(out_dir, "manifest.json"), "manifest.json")

    print("app_size   :", app_size)
    print("dat size   :", len(dat))
    print("zip        :", zip_path)
    print("version str in bin present:", b"DAHAOREN" in app)


if __name__ == "__main__":
    main()
