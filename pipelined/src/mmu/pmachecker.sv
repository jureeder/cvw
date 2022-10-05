///////////////////////////////////////////
// pmachecker.sv
//
// Written: tfleming@hmc.edu & jtorrey@hmc.edu 20 April 2021
// Modified: 
//
// Purpose: Examines all physical memory accesses and identifies attributes of
//          the memory region accessed.
//          Can report illegal accesses to the trap unit and cause a fault.
// 
// A component of the Wally configurable RISC-V project.
// 
// Copyright (C) 2021 Harvey Mudd College & Oklahoma State University
//
// MIT LICENSE
// Permission is hereby granted, free of charge, to any person obtaining a copy of this 
// software and associated documentation files (the "Software"), to deal in the Software 
// without restriction, including without limitation the rights to use, copy, modify, merge, 
// publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
// to whom the Software is furnished to do so, subject to the following conditions:
//
//   The above copyright notice and this permission notice shall be included in all copies or 
//   substantial portions of the Software.
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
//   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
//   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
//   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
//   OR OTHER DEALINGS IN THE SOFTWARE.
////////////////////////////////////////////////////////////////////////////////////////////////

`include "wally-config.vh"

module pmachecker (
//  input  logic        clk, reset, // *** unused in this module and all sub modules.

  input  logic [`PA_BITS-1:0] PhysicalAddress,
  input  logic [1:0]          Size,
  input  logic        AtomicAccessM, ExecuteAccessF, WriteAccessM, ReadAccessM, // *** atomicaccessM is unused but might want to stay in for future use.
  output logic        Cacheable, Idempotent, AtomicAllowed, SelTIM,
  output logic        PMAInstrAccessFaultF,
  output logic        PMALoadAccessFaultM,
  output logic        PMAStoreAmoAccessFaultM
);

  logic PMAAccessFault;
  logic AccessRW, AccessRWX, AccessRX;
  logic [10:0]  SelRegions;

  // Determine what type of access is being made
  assign AccessRW = ReadAccessM | WriteAccessM;
  assign AccessRWX = ReadAccessM | WriteAccessM | ExecuteAccessF;
  assign AccessRX = ReadAccessM | ExecuteAccessF;

  // Determine which region of physical memory (if any) is being accessed
  adrdecs adrdecs(PhysicalAddress, AccessRW, AccessRX, AccessRWX, Size, SelRegions);

  // Only non-core RAM/ROM memory regions are cacheable
  assign Cacheable = SelRegions[8] | SelRegions[7] | SelRegions[6];
  assign Idempotent = SelRegions[10] | SelRegions[9] | SelRegions[8] | SelRegions[6];
  assign AtomicAllowed = SelRegions[10] | SelRegions[9] | SelRegions[8] | SelRegions[6];
  assign SelTIM = SelRegions[10] | SelRegions[9];

  // Detect access faults
  assign PMAAccessFault = (SelRegions[0]) & AccessRWX;  
  assign PMAInstrAccessFaultF = ExecuteAccessF & PMAAccessFault;
  assign PMALoadAccessFaultM  = ReadAccessM    & PMAAccessFault;
  assign PMAStoreAmoAccessFaultM = WriteAccessM   & PMAAccessFault;
endmodule

