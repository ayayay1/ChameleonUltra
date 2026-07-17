# OTA 打包工具 (tools/ota)

为 ChameleonUltra 生成**可被第三方 DFU GUI 刷入**的合法 `dfu-cc` OTA 包，并对包做独立校验。

## 依赖
```bash
pip install ecdsa protobuf
```
`dfu_cc_pb2.py` 已随本目录一起提供（来自 nordicsemi dfu），无需额外安装 nrfutil。

## 生成 OTA 包
先编译固件（产物在 `firmware/objects/application.bin`），然后：
```bash
python generate.py            # 默认读取仓库内 application.bin，输出到 ./out/ultra-dfu-app.zip
python generate.py --out out  # 指定输出目录
python generate.py --app-bin /path/to/application.bin --pem /path/to/chameleon.pem
```
生成的 `out/` 目录含 `application.bin` / `application.dat` / `manifest.json` 及 `ultra-dfu-app.zip`，
可直接用任意支持 nRF DFU 的 GUI/CLI 刷入（app-only，不动 bootloader）。

## 校验 OTA 包
```bash
python verify.py                       # 校验 ./out/ultra-dfu-app.zip
python verify.py path/to/package.zip   # 校验指定包
```
校验项：dfu-cc 结构、fw/hw 版本、sd_req、type、app_size、SHA256(LE)、CRC boot_validation，
以及用项目 bootloader 公钥 `pk[64]` 验 ECDSA-P256 签名。

## 说明
- 签名密钥 `resource/dfu_key/chameleon.pem` 是项目官方 DFU 开发密钥（上游公共持有），
  其公钥固化在 bootloader 中，因此本工具产出的包可在 stock bootloader 上通过验签。
- 版本号由 `firmware/Makefile.defs` 的 `GIT_VERSION` 编译进 `application.bin`，
  本工具只负责打包，不修改版本串。
