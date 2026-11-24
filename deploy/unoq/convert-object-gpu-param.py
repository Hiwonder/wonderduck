#!/usr/bin/env python3
"""Fuse the two YOLO26n attention blocks for the UNO Q Vulkan runtime."""

import argparse
from pathlib import Path


# PNNX 20261020 exports YOLO26n's two attention blocks as MatMul and Softmax.
# NCNN has no Vulkan MatMul for those layers, but its SDPA layer implements
# the same equation. Each pair lists the expected input and the replacement.
# The first four rows also check the head shape and query/key/value ordering.
ATTENTION_BLOCKS = (
    (
        """
        Convolution conv_129 1 1 149 150 0=256 1=1 11=1 12=1 13=1 14=0 2=1 3=1 4=0 5=1 6=32768
        Reshape reshape_193 1 1 150 151 0=400 1=128 2=2
        Slice split_5 1 3 151 152 153 154 -23300=3,32,32,64 1=1
        Split splitncnn_22 1 2 154 155 156
        BinaryOp mul_14 1 1 152 157 0=2 1=1 2=0.176776692
        Permute permute_189 1 1 157 158 0=1
        MatMul matmul_205 2 1 158 153 159
        Softmax softmax_93 1 1 159 160 0=2 1=1
        MatMul matmultransb_0 2 1 156 160 161 0=1
        """,
        """
        Permute psa0_query 1 1 152 157 0=1
        Permute psa0_key 1 1 153 158 0=1
        Permute psa0_value 1 1 156 159 0=1
        SDPA psa0_attention 3 1 157 158 159 160
        Permute psa0_output 1 1 160 161 0=1
        """,
    ),
    (
        """
        Convolution conv_166 1 1 324 325 0=256 1=1 11=1 12=1 13=1 14=0 2=1 3=1 4=0 5=1 6=32768
        Reshape reshape_196 1 1 325 326 0=400 1=128 2=2
        Slice split_10 1 3 326 327 328 329 -23300=3,32,32,64 1=1
        Split splitncnn_46 1 2 329 330 331
        BinaryOp mul_32 1 1 327 332 0=2 1=1 2=0.176776692
        Permute permute_191 1 1 332 333 0=1
        MatMul matmul_207 2 1 333 328 334
        Softmax softmax_94 1 1 334 335 0=2 1=1
        MatMul matmultransb_1 2 1 331 335 336 0=1
        """,
        """
        Permute psa1_query 1 1 327 332 0=1
        Permute psa1_key 1 1 328 333 0=1
        Permute psa1_value 1 1 331 334 0=1
        SDPA psa1_attention 3 1 332 333 334 335
        Permute psa1_output 1 1 335 336 0=1
        """,
    ),
)


def convert(source: str) -> str:
    lines = source.splitlines()
    if len(lines) != 343 or lines[0].strip() != "7767517" or lines[1].split() != ["341", "415"]:
        raise ValueError("expected the NCNN YOLO26n graph exported by PNNX 20261020")

    # The Permute layers transpose each head between [channels, tokens] and
    # [tokens, channels]. SDPA supplies the original 1/sqrt(32) query scale.
    # Each replacement keeps the layer count, blob count, and .bin weight order.
    for expected, replacement in ATTENTION_BLOCKS:
        expected_layers = [line.split() for line in expected.strip().splitlines()]
        replacement_layers = [line.strip() for line in replacement.strip().splitlines()]
        matches = [
            index
            for index, line in enumerate(lines)
            if line.split() == expected_layers[0]
            and [item.split() for item in lines[index:index + len(expected_layers)]] == expected_layers
        ]
        if len(matches) != 1:
            raise ValueError(f"expected one YOLO26n attention block after {expected_layers[0][1]}")
        end = matches[0] + len(expected_layers)
        lines[end - len(replacement_layers):end] = replacement_layers

    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_param", type=Path, help="PNNX's NCNN .param output")
    parser.add_argument("output_param", type=Path, help="Vulkan-ready NCNN .param")
    args = parser.parse_args()
    try:
        converted = convert(args.input_param.read_text(encoding="utf-8"))
        args.output_param.write_text(converted, encoding="utf-8")
    except (OSError, ValueError) as error:
        parser.exit(1, f"{parser.prog}: {error}\n")


if __name__ == "__main__":
    main()
