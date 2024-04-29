///////////////////////////////////////////
// cacheLFSU.sv
//
// Written: Rose Thompson ross1728@gmail.com
// Created: 20 July 2021
// Modified: 20 January 2023
//
// Purpose: Implements Pseudo LRU. Tested for Powers of 2.
//
// Documentation: RISC-V System on Chip Design Chapter 7 (Figures 7.8 and 7.15 to 7.18)
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// https://github.com/openhwgroup/cvw
//
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////



// do we need to include any of the other inputs/outputs that were part of the cacheLRU module?


module cacheLFSR #(parameter WIDTH = 5) ( input logic clk, rst, 
              output logic [WIDTH - 1:0] lfsrResult); //variable length
 
logic [WIDTH - 1:0] curr; // variable length
logic       next;

flopenl #(WIDTH) ff (.clk, .load(rst), .en(1'b1), .d({next, curr[WIDTH - 1:1]}), .val({{(WIDTH - 1){1'b0}}, 1'b1}), .q(curr));
assign lfsrResult = curr;

if (WIDTH == 3) begin
    assign next = curr[2] ^ curr[0]; 
end else if (WIDTH == 4) begin
    assign next = curr[3] ^ curr[0];
end else if (WIDTH == 5) begin
    assign next = curr[4] ^ curr[3] ^ curr[2] ^ curr[0];
end else if (WIDTH == 6) begin
    assign next = curr[5] ^ curr[4] ^ curr[2] ^ curr[1];
end else if (WIDTH == 7) begin
    assign next = curr[6] ^ curr[5] ^ curr[3] ^ curr[0];
end else if (WIDTH == 8) begin
    assign next = curr[7] ^ curr[5] ^ curr[2] ^ curr[1];
end else if (WIDTH == 9) begin
    assign next = curr[8] ^ curr[6] ^ curr[5] ^ curr[4] ^ curr[3] ^ curr[2];
end

endmodule

module cacheLRU
  #(parameter NUMWAYS = 4, SETLEN = 9, OFFSETLEN = 5, NUMLINES = 128) (
  input  logic                clk, 
  input  logic                reset,
  input  logic                FlushStage,
  input  logic                CacheEn,         // Enable the cache memory arrays.  Disable hold read data constant
  input  logic [NUMWAYS-1:0]  HitWay,          // Which way is valid and matches PAdr's tag
  input  logic [NUMWAYS-1:0]  ValidWay,        // Which ways for a particular set are valid, ignores tag
  input  logic [SETLEN-1:0]   CacheSetData,    // Cache address, the output of the address select mux, NextAdr, PAdr, or FlushAdr
  input  logic [SETLEN-1:0]   CacheSetTag,     // Cache address, the output of the address select mux, NextAdr, PAdr, or FlushAdr
  input  logic [SETLEN-1:0]   PAdr,            // Physical address 
  input  logic                LRUWriteEn,      // Update the LRU state
  input  logic                SetValid,        // Set the dirty bit in the selected way and set
  input  logic                ClearValid,      // Clear the dirty bit in the selected way and set
  input  logic                InvalidateCache, // Clear all valid bits
  output logic [NUMWAYS-1:0]  VictimWay        // LRU selects a victim to evict
);

  localparam                           LOGNUMWAYS = $clog2(NUMWAYS);
  localparem                           LFSRWIDTH = LOGNUMWAYS + 2; // added

  logic [NUMWAYS-2:0]                  LRUMemory [NUMLINES-1:0];
  //logic [NUMWAYS-2:0]                  CurrLRU;
  //logic [NUMWAYS-2:0]                  NextLRU;
  logic [LOGNUMWAYS-1:0]               HitWayEncoded, Way;
  logic                                AllValid;

  logic [LFSRWIDTH - 1:0]              currLFSR; // variable length, added
  logic                                nextLFSR; // 1 bit, added

  flopenl #(LFSRWIDTH) ff (.clk, .load(rst), .en(CacheEn), .d({next, currLFSR[LFSRWIDTH - 1:1]}), .val({{(LFSRWIDTH - 1){1'b0}}, 1'b1}), .q(currLFSR)); // replaced enabler with CacheEN
  assign lfsrResult = currLFSR;

  if (LFSRWIDTH == 3) begin
    assign nextLFSR = currLFSR[2] ^ currLFSR[0]; 
  end else if (LFSRWIDTH == 4) begin
    assign nextLFSR = currLFSR[3] ^ currLFSR[0];
  end else if (LFSRWIDTH == 5) begin
    assign nextLFSR = currLFSR[4] ^ currLFSR[3] ^ currLFSR[2] ^ currLFSR[0];
  end else if (LFSRWIDTH == 6) begin
    assign nextLFSR = currLFSR[5] ^ currLFSR[4] ^ currLFSR[2] ^ currLFSR[1];
  end else if (LFSRWIDTH == 7) begin
    assign nextLFSR = currLFSR[6] ^ currLFSR[5] ^ currLFSR[3] ^ currLFSR[0];
  end else if (LFSRWIDTH == 8) begin
    assign nextLFSR = currLFSR[7] ^ currLFSR[5] ^ currLFSR[2] ^ currLFSR[1];
  end else if (LFSRWIDTH == 9) begin
    assign nextLFSR = currLFSR[8] ^ currLFSR[6] ^ currLFSR[5] ^ currLFSR[4] ^ currLFSR[3] ^ currLFSR[2];
  end

  /* verilator lint_off UNOPTFLAT */
  // Rose: For some reason verilator does not like this.  I checked and it is not a circular path.
  //logic [NUMWAYS-2:0]                  LRUUpdate;
  //logic [LOGNUMWAYS-1:0] Intermediate [NUMWAYS-2:0];
  /* verilator lint_on UNOPTFLAT */

  logic [NUMWAYS-1:0] FirstZero;
  logic [LOGNUMWAYS-1:0] FirstZeroWay;
  logic [LOGNUMWAYS-1:0] VictimWayEnc;

  binencoder #(NUMWAYS) hitwayencoder(HitWay, HitWayEncoded);

  assign AllValid = &ValidWay;
 
  priorityonehot #(NUMWAYS) FirstZeroEncoder(~ValidWay, FirstZero);
  binencoder #(NUMWAYS) FirstZeroWayEncoder(FirstZero, FirstZeroWay);
  mux2 #(LOGNUMWAYS) VictimMux(FirstZeroWay, lfsrResult, AllValid, VictimWayEnc); // check LFSR size
  decoder #(LOGNUMWAYS) decoder (VictimWayEnc, VictimWay);

endmodule


