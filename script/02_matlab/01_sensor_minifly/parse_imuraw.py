#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Parse imuraw.txt and write imuraw.csv with headers:
  a_x,a_y,a_z,v_x,v_y,v_z,time_stamp

Each line format (hex, big-endian):
AAAAF11C <6x float32> <1x uint32> <1x checksum byte>

Usage:
  python parse_imuraw.py [input_file] [output_file]
Defaults:
  input_file  = imuraw.txt (same directory)
  output_file = imuraw.csv (same directory)
"""

import csv
import os
import struct
import sys
from typing import List, Optional

HEADER_TOKEN = "AAAAF11C"
DEFAULT_INPUT = "imuraw.txt"
DEFAULT_OUTPUT = "imuraw.csv"


def _hex_to_be_float32(word: str) -> float:
	bytes_be = bytes.fromhex(word)
	return struct.unpack(">f", bytes_be)[0]


def _hex_to_be_u32(word: str) -> int:
	bytes_be = bytes.fromhex(word)
	return struct.unpack(">I", bytes_be)[0]


def _parse_line(line: str) -> Optional[List[object]]:
	line = line.strip()
	if not line or line.startswith("#"):
		return None
	# split by whitespace
	tokens = line.split()
	if not tokens:
		return None
	# find header index (normally 0)
	header_idx = -1
	for idx, tok in enumerate(tokens):
		if tok.upper() == HEADER_TOKEN:
			header_idx = idx
			break
	# if not found, assume the first token is header
	if header_idx == -1:
		header_idx = 0
	# need at least: header + 6 floats + 1 u32 (checksum is optional to parse)
	needed = header_idx + 1 + 6 + 1
	if len(tokens) < needed:
		return None
	# extract payload fields
	float_hex = tokens[header_idx + 1 : header_idx + 1 + 6]
	ts_hex = tokens[header_idx + 1 + 6]
	# all words should be 8 hex chars
	for hx in float_hex + [ts_hex]:
		if len(hx) != 8:
			return None
	try:
		floats = [_hex_to_be_float32(hx) for hx in float_hex]
		ts = _hex_to_be_u32(ts_hex)
		return [
			floats[0],
			floats[1],
			floats[2],
			floats[3],
			floats[4],
			floats[5],
			ts,
		]
	except Exception:
		return None


def parse_file(input_path: str, output_path: str) -> None:
	rows: List[List[object]] = []
	with open(input_path, "r", encoding="utf-8") as f:
		for line_no, line in enumerate(f, start=1):
			parsed = _parse_line(line)
			if parsed is None:
				continue
			rows.append(parsed)
	# write CSV
	headers = ["a_x", "a_y", "a_z", "v_x", "v_y", "v_z", "time_stamp"]
	with open(output_path, "w", newline="", encoding="utf-8") as csvfile:
		writer = csv.writer(csvfile)
		writer.writerow(headers)
		writer.writerows(rows)


if __name__ == "__main__":
	# default files in the same directory
	script_dir = os.path.dirname(os.path.abspath(__file__))
	input_file = os.path.join(script_dir, DEFAULT_INPUT)
	output_file = os.path.join(script_dir, DEFAULT_OUTPUT)
	# CLI overrides
	if len(sys.argv) >= 2:
		input_file = sys.argv[1]
		if not os.path.isabs(input_file):
			input_file = os.path.join(script_dir, input_file)
	if len(sys.argv) >= 3:
		output_file = sys.argv[2]
		if not os.path.isabs(output_file):
			output_file = os.path.join(script_dir, output_file)
	# run
	if not os.path.exists(input_file):
		raise FileNotFoundError(f"Input file not found: {input_file}")
	parse_file(input_file, output_file)
	print(f"Done. Wrote {os.path.getsize(output_file)} bytes to: {output_file}") 