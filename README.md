# ParaLarPD: Parallel FPGA Router

## Requirements

- C++ compiler with C++11 standard support
- Boost C++ library (version 1.60)
    - Install via package manager (e.g., `apt-get install libboost-all-dev`) or from [www.boost.org](https://www.boost.org)
- Intel TBB library
    - Download and extract from [https://www.threadingbuildingblocks.org/download\#stable-releases](https://www.threadingbuildingblocks.org/download#stable-releases)

***

## Compilation Steps

1. `cd build`
2. Set the environment variable for the Intel TBB **include** directory:
`export TBB_INC_DIR=<path to Intel TBB include directory>`
_Example:_
`export TBB_INC_DIR=/home/user/tbb/include`
3. Set the environment variable for the Intel TBB **library** directory:
`export TBB_LIB_DIR=<path to Intel TBB library directory>`
_Example:_
`export TBB_LIB_DIR=/home/user/tbb/lib/intel64/gcc4.7`
4. Run CMake to generate makefiles:
`cmake ..`
5. Compile the project:
`make`
_(For verbose output: `make VERBOSE=1`)_
    - Use `make clean` to clean the build

The final executable (`Router`) will be located in the `build` subdirectory after a successful build.

***

## Preparing Netlists Using VTR

- The required netlist must be compiled using VTR:
[VTR (VPR) Documentation](http://www.eecg.utoronto.ca/vtr/terms.html)
- Command format:

```
./vpr <arch.xml> <benchmark_name> --pack --place
```

_Example:_

```
./vpr fpga.xml ex5p --pack --place
```

This command packs and places the benchmark (`ex5p.blif`) into the given FPGA architecture (`fpga.xml`) to produce `ex5p.net` and `ex5p.place` files which are required by the router.
- Timing-Driven Router (Default):

```
./vpr fpga.xml ex5p --pack --place --route
```

- Routing-Driven Router:

```
./vpr fpga.xml ex5p --pack --place --route --router_algorithm breadth_first
```


Example benchmarks and architecture files are provided in the `examples` directory. MCNC .blif benchmarks are included and need to be compiled using VTR as above.

***

## Running the Router

1. After compilation, the `Router` executable will be in the `build` directory.
2. Copy the `Router` executable to the `examples` directory (which contains all required input files).
3. Run the router with:

```
./Router <arch.xml> <benchmark_name> <num_threads> <num_of_iterations> <max_channel_capacity>
```

_Example:_

```
./Router fpga.xml ex5p 4 50 100
```

    - This runs the router on the `ex5p` benchmark with a channel width of 100, using 4 threads for 50 iterations.
    - Ensure all files: `fpga.xml`, `ex5p.blif`, `ex5p.net`, `ex5p.place` are present in the same directory as `Router`.

#### Debugging Example

To debug using gdb:

```
gdb --args ./Router <arch.xml> <benchmark_name> <num_threads> <num_of_iterations> <max_channel_capacity>
```


***

## Citation

If you use this code in your research, please cite:

```
@article{agrawal2019paralarpd,
  title={ParaLarPD: Parallel FPGA router using primal-dual sub-gradient method},
  author={Agrawal, Rohit and Ahuja, Kapil and Hau Hoo, Chin and Duy Anh Nguyen, Tuan and Kumar, Akash},
  journal={Electronics},
  volume={8},
  number={12},
  pages={1439},
  year={2019},
  publisher={MDPI}
}
```

