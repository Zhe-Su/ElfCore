//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// ___ Timescale ___ ///
`timescale 1ps / 1ps
// Minimal specs for PSP build

// Synthesis/simulation switches used in submodules
`define SINTESI_RTL

`define notDelay 35
`define andDelay 55
`define orDelay 55
`define nandDelay 15
`define norDelay 50
`define xorDelay 60
`define xnorDelay 45
`define or3Delay 70
`define nor3Delay 65
`define xor3Delay 80
`define DLatchDelay 75
`define DLatchDelayR 80
`define FlipFlopDelayR 90
`define mullerCDelay 70

`define OCCUPANCY 4
`define mux2x1Delay 100
`define mux3x1Delay 120
`define mux4x1Delay 140
`define mux5x1Delay 160
`define mux6x1Delay 180
`define mux7x1Delay 200
`define mux8x1Delay 220
