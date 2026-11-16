# SPDX-License-Identifier: Apache-2.0
# Gate 0 브리지 처리량 시험 펌웨어 (Gate 0 USB-bridge throughput test firmware)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bridge_test)

target_sources(app PRIVATE src/main.c)
