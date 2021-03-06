/*BEGIN_LEGAL 
  Intel Open Source License 

  Copyright (c) 2002-2018 Intel Corporation. All rights reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.  Redistributions
  in binary form must reproduce the above copyright notice, this list of
  conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.  Neither the name of
  the Intel Corporation nor the names of its contributors may be used to
  endorse or promote products derived from this software without
  specific prior written permission.
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL OR
  ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  END_LEGAL */
/*! @file
 *  This file contains an ISA-portable cache simulator
 *  data cache hierarchies
 */


#include "pin.H"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>

#include "dcache.H"
#include "pin_profile.H"
#include "instlib.H"
#include "filter.H"

#define INTEL_CACHE_SIZE     ((64 * MEGA)/16)
#define INTEL_CACHELINE_SIZE  64
#define INTEL_ASSOCIATIVITY   8

std::ofstream outFile;
INSTLIB::FILTER_RTN filter;

/* ===================================================================== */
/* Commandline Switches */
/* ===================================================================== */

KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE,    "pintool",
			    "o", "hbpintool.out", "specify hbpintool file name");
KNOB<BOOL>   KnobTrackLoads(KNOB_MODE_WRITEONCE,    "pintool",
			    "tl", "0", "track individual loads -- increases profiling time");
KNOB<BOOL>   KnobTrackStores(KNOB_MODE_WRITEONCE,   "pintool",
			     "ts", "0", "track individual stores -- increases profiling time");
KNOB<UINT32> KnobThresholdHit(KNOB_MODE_WRITEONCE , "pintool",
			      "rh", "100", "only report memops with hit count above threshold");
KNOB<UINT32> KnobThresholdMiss(KNOB_MODE_WRITEONCE, "pintool",
			       "rm","100", "only report memops with miss count above threshold");
KNOB<UINT32> KnobCacheSize(KNOB_MODE_WRITEONCE, "pintool",
			   "c","32", "cache size in kilobytes");
KNOB<UINT32> KnobLineSize(KNOB_MODE_WRITEONCE, "pintool",
			  "b","32", "cache block size in bytes");
KNOB<UINT32> KnobAssociativity(KNOB_MODE_WRITEONCE, "pintool",
			       "a","4", "cache associativity (1 for direct mapped)");

KNOB<BOOL> KnobColdOnly(KNOB_MODE_WRITEONCE, "pintool",
			"co", "1", "only count 'cold' cache misses");

KNOB<std::string> KnobRtnEpochMarker(KNOB_MODE_WRITEONCE, "pintool",
                                     "epoch_marker", "", "Routine to set as the epoch marker");

typedef std::pair<UINT32, double> CLSIZE_WATTS_PAIR;
typedef std::pair<UINT32, double> CLSIZE_LATENCY_PAIR;
typedef std::map<UINT32, double>  POWER_MAP;   // access size => power in watts
typedef std::map<UINT32, double>  LATENCY_MAP; // access size => latency in ns

#define array_size(x)				\
    (sizeof(x)/sizeof(x[0]))

/* ===================================================================== */
/* Print Help Message                                                    */
/* ===================================================================== */

INT32 Usage()
{
    cerr <<
        "This tool represents a cache simulator.\n"
        "\n";

    cerr << KNOB_BASE::StringKnobSummary() << endl; 
    return -1;
}

/* ===================================================================== */
/* Global Variables */
/* ===================================================================== */

// wrap configuation constants into their own name space to avoid name clashes
namespace DL1
{
    const UINT32 max_sets = 128*KILO; // cacheSize / (lineSize * associativity);
    const UINT32 max_associativity = 256; // associativity;
    const CACHE_ALLOC::STORE_ALLOCATION allocation = CACHE_ALLOC::STORE_ALLOCATE;

    typedef CACHE_ROUND_ROBIN(max_sets, max_associativity, allocation) CACHE_INTEL;
    typedef CACHE_ROUND_ROBIN_INFINITE(max_sets, max_associativity, allocation) CACHE_HAMMERBLADE;
}

DL1::CACHE_HAMMERBLADE* dl1 = NULL;

DL1::CACHE_INTEL *dl1_intel = NULL;

typedef enum
{
    COUNTER_MISS = 0,
    COUNTER_HIT = 1,
    COUNTER_NUM
} COUNTER;



typedef  COUNTER_ARRAY<UINT64, COUNTER_NUM> COUNTER_HIT_MISS;
typedef std::vector<DL1::CACHE_HAMMERBLADE*>DL1_ARRAY;

// holds the counters with misses and hits
// conceptually this is an array indexed by instruction address
// COMPRESSOR_COUNTER<ADDRINT, UINT32, COUNTER_HIT_MISS> profile;
// COMPRESSOR_COUNTER<ADDRINT, UINT32, COUNTER_HIT_MISS> profile_intel;

DL1_ARRAY hammerblade_dl1s;


typedef std::vector<UINT64> ICOUNTER;

/* indexed by COUNTER */
ICOUNTER hammerblade_icount(COUNTER_NUM, 0);
ICOUNTER intel_icount(COUNTER_NUM,0);



/* ===================================================================== */

static void ResetDl1(void)
{    
    dl1 = new DL1::CACHE_HAMMERBLADE("HammerBlade L1",
				     KnobCacheSize.Value() * KILO,
				     KnobLineSize.Value(),
				     KnobAssociativity.Value());
    hammerblade_dl1s.push_back(dl1);
}

void Routine(RTN rtn, void *v)
{
    if (!filter.SelectRtn(rtn))
	return;
    
    // reset dl1 every time the epoch marker is called
    if (RTN_Name(rtn) == KnobRtnEpochMarker.Value()) {
	RTN_Open(rtn);
	RTN_InsertCall(rtn, IPOINT_BEFORE, (AFUNPTR) ResetDl1, IARG_END);
	RTN_Close(rtn);
    }	
}

static
VOID CountInstruction(bool intel_hit, bool hammerblade_hit)
{
    intel_icount[intel_hit ? COUNTER_HIT : COUNTER_MISS]++;
    hammerblade_icount[hammerblade_hit ? COUNTER_HIT : COUNTER_MISS]++;
}

static
VOID LoadStoreInstruction(ADDRINT read_addr,  UINT32 read_size,
			  ADDRINT write_addr, UINT32 write_size)
{
    bool intel_hit, hammerblade_hit;

    intel_hit = dl1_intel->Access(read_addr, read_size, CACHE_BASE::ACCESS_TYPE_LOAD);
    hammerblade_hit = dl1->Access(read_addr, read_size, CACHE_BASE::ACCESS_TYPE_LOAD);

    intel_hit &= dl1_intel->Access(write_addr, write_size, CACHE_BASE::ACCESS_TYPE_STORE);
    hammerblade_hit &= dl1->Access(write_addr, write_size, CACHE_BASE::ACCESS_TYPE_STORE);

    CountInstruction(intel_hit, hammerblade_hit);
}

static
VOID LoadInstruction(ADDRINT read_addr, UINT32 read_size)
{
    bool intel_hit, hammerblade_hit;

    intel_hit = dl1_intel->Access(read_addr, read_size, CACHE_BASE::ACCESS_TYPE_LOAD);
    hammerblade_hit = dl1->Access(read_addr, read_size, CACHE_BASE::ACCESS_TYPE_LOAD);

    CountInstruction(intel_hit, hammerblade_hit);
}

static
VOID StoreInstruction(ADDRINT write_addr, UINT32 write_size)
{
    bool intel_hit, hammerblade_hit;

    intel_hit = dl1_intel->Access(write_addr, write_size, CACHE_BASE::ACCESS_TYPE_STORE);
    hammerblade_hit = dl1->Access(write_addr, write_size, CACHE_BASE::ACCESS_TYPE_STORE);

    CountInstruction(intel_hit, hammerblade_hit);
}

static
VOID NonMemoryInstruction()
{
    CountInstruction(true, true);
}

/* ===================================================================== */

VOID Instruction(INS ins, void * v)
{
    RTN rtn = INS_Rtn(ins);
    if (!RTN_Valid(rtn))
	return;
    
    if (!filter.SelectRtn(rtn))
    	return;
    
    bool is_memory_read, is_memory_write;

    is_memory_read  = INS_IsMemoryRead(ins) && INS_IsStandardMemop(ins);
    is_memory_write = INS_IsMemoryWrite(ins) && INS_IsStandardMemop(ins);
    if (is_memory_read && is_memory_write) {
	INS_InsertPredicatedCall(
	    ins, IPOINT_BEFORE, (AFUNPTR) LoadStoreInstruction,
	    IARG_MEMORYREAD_EA,
	    IARG_MEMORYREAD_SIZE,
	    IARG_MEMORYWRITE_EA,
	    IARG_MEMORYWRITE_SIZE,
	    IARG_END);	
    } else if (is_memory_read) {
	// for instructions that read
	INS_InsertPredicatedCall(
	    ins, IPOINT_BEFORE, (AFUNPTR) LoadInstruction,
	    IARG_MEMORYREAD_EA,
	    IARG_MEMORYREAD_SIZE,
	    IARG_END);	
    } else if (is_memory_write) {
	// for instructions that write
	INS_InsertPredicatedCall(
	    ins, IPOINT_BEFORE, (AFUNPTR) StoreInstruction,
	    IARG_MEMORYWRITE_EA,
	    IARG_MEMORYWRITE_SIZE,
	    IARG_END);
    } else {
	// for non memory instructions
	INS_InsertPredicatedCall(
	    ins, IPOINT_BEFORE, (AFUNPTR) NonMemoryInstruction,
	    IARG_END);
    }        
}

static void HBPintoolFini(int code, void *v)
{
    const int prefix_width = 16;
    std::string hammerblade_prefix = "HammerBlade";
    std::string xeon_prefix        = "Xeon E7-8894 v4";
    
    // instruction energy costs
    double Xeon_JPInstruction = 5.7e-9; // 3 nops per cycle
    double HammerBlade_JPInstruction = 5e-12;

    // Xeon IPC = 1/2?
    // Manycore IPC = 512 (or something?)
    
    // memory energy costs
    //double DDR4_JPBit = 348e-12; // if we just stream
    double DDR4_JPBit = 124.07e-12;
    double HBM2_JPBit = 3.6e-12;

    double Joules_hammerblade_insts = HammerBlade_JPInstruction * (hammerblade_icount[COUNTER_HIT]+hammerblade_icount[COUNTER_MISS]);
    double Joules_hammerblade_membits = HBM2_JPBit * dl1->LineSize() * 8 * hammerblade_icount[COUNTER_MISS];
    double Joules_hammerblade = Joules_hammerblade_insts + Joules_hammerblade_membits;           

    double Joules_xeon_insts =  Xeon_JPInstruction * (intel_icount[COUNTER_HIT]+intel_icount[COUNTER_MISS]);
    double Joules_xeon_membits = (DDR4_JPBit * INTEL_CACHELINE_SIZE * 8 * intel_icount[COUNTER_MISS]);
    double Joules_xeon = Joules_xeon_insts + Joules_xeon_membits;             

    // // performance
    // double Time_DRAM_Xeon = 60e-9; // 60ns
    // double Time_DRAM_Hammerblade = (0.116e-9)*16; // 0.116ns
    
    // outFile << std::setw(prefix_width) << hammerblade_prefix << " Estimated Performance: " << std::scientific << Time_DRAM_Hammerblade * hammerblade_icount[COUNTER_MISS] << "\n";
    // outFile << std::setw(prefix_width) << xeon_prefix        << " Estimated Performance: " << std::scientific << Time_DRAM_Xeon * intel_icount[COUNTER_MISS]              << "\n";
    // // IPC, HZ => Instructions / (IPC * HZ) = Seconds
    // // Take Joules/Seconds
    
    // // outFile << std::setw(prefix_width) << hammerblade_prefix << ": "
    // //         << "Instructions: " << std::scientific << (double) (hammerblade_icount[COUNTER_HIT]+hammerblade_icount[COUNTER_MISS]) << ", "
    // //         << "DRAM Bits: "   << std::scientific << (double) (hammerblade_icount[COUNTER_MISS] * dl1->LineSize() * 8) << ", "
    // //         << "Misses: "      << std::scientific << (double) (hammerblade_icount[COUNTER_MISS])
    // //         << "\n";

    // // outFile << std::setw(prefix_width) << xeon_prefix << ": "
    // //         << "Instructions: " << std::scientific << (double) (intel_icount[COUNTER_HIT]+intel_icount[COUNTER_MISS]) << ", "
    // //         << "DRAM Bits: "   << std::scientific << (double) (intel_icount[COUNTER_MISS] * INTEL_CACHELINE_SIZE * 8) << ", "
    // //         << "Misses: "      << std::scientific << (double) (intel_icount[COUNTER_MISS])
    // //         << "\n";
    
    // outFile << std::setw(prefix_width) << hammerblade_prefix << " Energy Cost: " << std::scientific << Joules_hammerblade << " J\n";
    // // outFile << std::setw(prefix_width) << hammerblade_prefix << ": "
    // //         << "Ins Energy Cost (%): "  << std::fixed << (Joules_hammerblade_insts/Joules_hammerblade)*100 << ", "
    // //         << "Mem Energy Cost (%): "  << std::fixed << (Joules_hammerblade_membits/Joules_hammerblade)*100
    // //         << "\n";
    
    
    // outFile << std::setw(prefix_width) << xeon_prefix << " Energy Cost: " << std::scientific << Joules_xeon << " J\n";
    // // outFile << std::setw(prefix_width) << xeon_prefix << ": "
    // //         << "Ins Energy Cost (%): " << std::fixed << ((double)Joules_xeon_insts/Joules_xeon)*100 << ", "
    // //         << "Mem Energy Cost (%): " << std::fixed << ((double)Joules_xeon_membits/Joules_xeon)*100
    // //         << "\n";

    double hammerblade_gop = (hammerblade_icount[COUNTER_HIT] + hammerblade_icount[COUNTER_MISS])/1e9;
    double intel_gop       = (intel_icount[COUNTER_HIT] + intel_icount[COUNTER_MISS])/1e9;
    
    outFile << std::setw(prefix_width) << hammerblade_prefix << ": "
            << std::scientific << hammerblade_gop / Joules_hammerblade << " GOPS/Watt\n";

    outFile << std::setw(prefix_width) << xeon_prefix << ": "
            << std::scientific <<  intel_gop / Joules_xeon << " GOPS/Watt\n";
    
    // outFile << "Energy Cost Ratio (" << xeon_prefix << "/" << hammerblade_prefix << "): " << std::fixed << Joules_xeon/Joules_hammerblade << "\n";
}

VOID Fini(int code, VOID * v)
{
    // OriginalFini(code, v);
    HBPintoolFini(code, v);
    outFile.close();
}
/* ===================================================================== */
/* Main                                                                  */
/* ===================================================================== */


int main(int argc, char *argv[])
{
    PIN_InitSymbols();

    if( PIN_Init(argc,argv) )
    {
        return Usage();
    }

    outFile.open(KnobOutputFile.Value().c_str());

    dl1_intel = new DL1::CACHE_INTEL("L1 Data Cache",
				     INTEL_CACHE_SIZE, // size
				     INTEL_CACHELINE_SIZE, // block size
				     INTEL_ASSOCIATIVITY); // associativity
    
    ResetDl1();   

    RTN_AddInstrumentFunction(Routine, NULL);
    
    // profile.SetKeyName("iaddr          ");
    // profile.SetCounterName("dcache:miss        dcache:hit");

    COUNTER_HIT_MISS threshold;

    threshold[COUNTER_HIT] = KnobThresholdHit.Value();
    threshold[COUNTER_MISS] = KnobThresholdMiss.Value();
    
    // profile.SetThreshold( threshold );

    filter.Activate();
    
    INS_AddInstrumentFunction(Instruction, 0);
    PIN_AddFiniFunction(Fini, 0);

    // Never returns

    PIN_StartProgram();
    
    return 0;
}

/* ===================================================================== */
/* eof */
/* ===================================================================== */
