# 梯户模拟器的常用动作。全部产物都是静态文件，没有服务端。

WASM_DIR := target/wasm32-unknown-unknown/release-wasm
WEB_WASM := web/tihu.wasm
PORT := 8017

.PHONY: all web wasm dist single serve test smoke fmt clean

all: web

## 编译 WASM 绑定，并把产物摆到 web/ 下
wasm:
	cargo build -p tihu-wasm --profile release-wasm --target wasm32-unknown-unknown
	cp $(WASM_DIR)/tihu_wasm.wasm $(WEB_WASM)
	@ls -l $(WEB_WASM)

## 给静态托管用的平铺版。把 dist/ 整个目录扔到任何静态服务器上就能跑
dist: wasm
	mkdir -p dist
	cp web/index.html web/app.js web/charts.js web/merge.js web/worker.js web/styles.css web/tihu.wasm dist/

## 单文件版。断网双击即开，见 AGENTS.md 第 7 节那条红线
single: wasm
	mkdir -p dist
	node tools/embed.js

web: wasm dist single
	@echo ""
	@echo "web/          开发用，需要一个静态服务器"
	@echo "dist/         平铺版，扔到静态托管上"
	@echo "dist/tihu.html 单文件版，双击就能开"

serve: wasm dist
	python3 -m http.server -d web $(PORT)

## ABI 冒烟：无导入项、可复现、不漏内存、坏输入不 trap
smoke: wasm
	node tools/smoke.js

test:
	cargo test --workspace --all-targets

fmt:
	cargo fmt --all

clean:
	cargo clean
	rm -rf dist
