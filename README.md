# RTBVH
A fast BVH library written in Rust. This library contains implementations of the following BVH building algorithms:
- SAH with Spatial splits (Highest quality, slow build times)
- Binned SAH (High quality, decently fast)
- Locally Ordered Clustering Builder (Very fast build times)

All building algorithms make efficient use of multiple threads (the best way they possibly can).

An MBVH implementation is included as well and all the BVH structures implement traversal algorithms for both single rays and quad rays.

This library emits a C/C++ library which is used in my [hobby renderer](https://github.com/meirbon/rendering-fw).

# Planning
- Benchmark
- Add support for packets larger than 4
- Increase coverage of API for FFI