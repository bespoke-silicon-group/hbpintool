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

#define INTEL_CACHE_SIZE     (2 * MEGA)
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
COMPRESSOR_COUNTER<ADDRINT, UINT32, COUNTER_HIT_MISS> profile;
COMPRESSOR_COUNTER<ADDRINT, UINT32, COUNTER_HIT_MISS> profile_intel;

DL1_ARRAY hammerblade_dl1s;
static UINT32 dl1_reset_count(void) { return hammerblade_dl1s.size(); }

/* ===================================================================== */

VOID LoadMulti(ADDRINT addr, UINT32 size, UINT32 instId)
{
    // first level D-cache
    const BOOL dl1Hit = dl1->Access(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD);
    
    const COUNTER counter = dl1Hit ? COUNTER_HIT : COUNTER_MISS;    
    profile[instId][counter]++;

    const BOOL dl1IntelHit = dl1_intel->Access(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD);    
    const COUNTER intelCounter = dl1IntelHit ? COUNTER_HIT : COUNTER_MISS;
    profile_intel[instId][intelCounter]++;
}

/* ===================================================================== */

VOID StoreMulti(ADDRINT addr, UINT32 size, UINT32 instId)
{
    // first level D-cache
    const BOOL dl1Hit = dl1->Access(addr, size, CACHE_BASE::ACCESS_TYPE_STORE);

    const COUNTER counter = dl1Hit ? COUNTER_HIT : COUNTER_MISS;
    profile[instId][counter]++;

    const BOOL dl1IntelHit = dl1_intel->Access(addr, size, CACHE_BASE::ACCESS_TYPE_STORE);    
    const COUNTER intelCounter = dl1IntelHit ? COUNTER_HIT : COUNTER_MISS;
    profile_intel[instId][intelCounter]++;
}

/* ===================================================================== */

VOID LoadSingle(ADDRINT addr, UINT32 instId)
{
    // @todo we may access several cache lines for 
    // first level D-cache
    const BOOL dl1Hit = dl1->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_LOAD);

    const COUNTER counter = dl1Hit ? COUNTER_HIT : COUNTER_MISS;
    profile[instId][counter]++;

    const BOOL dl1IntelHit = dl1_intel->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_LOAD);    
    const COUNTER intelCounter = dl1IntelHit ? COUNTER_HIT : COUNTER_MISS;
    profile_intel[instId][intelCounter]++;
}
/* ===================================================================== */

VOID StoreSingle(ADDRINT addr, UINT32 instId)
{
    // @todo we may access several cache lines for 
    // first level D-cache
    const BOOL dl1Hit = dl1->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_STORE);

    const COUNTER counter = dl1Hit ? COUNTER_HIT : COUNTER_MISS;
    profile[instId][counter]++;

    const BOOL dl1IntelHit = dl1_intel->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_STORE);    
    const COUNTER intelCounter = dl1IntelHit ? COUNTER_HIT : COUNTER_MISS;
    profile_intel[instId][intelCounter]++;
}

/* ===================================================================== */

VOID LoadMultiFast(ADDRINT addr, UINT32 size)
{
    dl1->Access(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD);
    dl1_intel->Access(addr, size, CACHE_BASE::ACCESS_TYPE_LOAD);
}

/* ===================================================================== */

VOID StoreMultiFast(ADDRINT addr, UINT32 size)
{
    dl1->Access(addr, size, CACHE_BASE::ACCESS_TYPE_STORE);
    dl1_intel->Access(addr, size, CACHE_BASE::ACCESS_TYPE_STORE);
}

/* ===================================================================== */

VOID LoadSingleFast(ADDRINT addr)
{
    dl1->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_LOAD);
    dl1_intel->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_LOAD);
}

/* ===================================================================== */

VOID StoreSingleFast(ADDRINT addr)
{
    dl1->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_STORE);
    dl1_intel->AccessSingleLine(addr, CACHE_BASE::ACCESS_TYPE_STORE);
}



/* ===================================================================== */

VOID Instruction(INS ins, void * v)
{
    RTN rtn = INS_Rtn(ins);
    if (!RTN_Valid(rtn))
	return;
    
    if (!filter.SelectRtn(rtn))
    	return;
    
    if (INS_IsMemoryRead(ins) && INS_IsStandardMemop(ins))
    {
        // map sparse INS addresses to dense IDs
        const ADDRINT iaddr = INS_Address(ins);
        const UINT32 instId = profile.Map(iaddr);

        const UINT32 size = INS_MemoryReadSize(ins);
        const BOOL   single = (size <= 4);
                
        if( KnobTrackLoads )
        {
            if( single )
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE, (AFUNPTR) LoadSingle,
                    IARG_MEMORYREAD_EA,
                    IARG_UINT32, instId,
                    IARG_END);
            }
            else
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) LoadMulti,
                    IARG_MEMORYREAD_EA,
                    IARG_MEMORYREAD_SIZE,
                    IARG_UINT32, instId,
                    IARG_END);
            }
                
        }
        else
        {
            if( single )
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) LoadSingleFast,
                    IARG_MEMORYREAD_EA,
                    IARG_END);
                        
            }
            else
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) LoadMultiFast,
                    IARG_MEMORYREAD_EA,
                    IARG_MEMORYREAD_SIZE,
                    IARG_END);
            }
        }
    }
        
    if ( INS_IsMemoryWrite(ins) && INS_IsStandardMemop(ins))
    {
        // map sparse INS addresses to dense IDs
        const ADDRINT iaddr = INS_Address(ins);
        const UINT32 instId = profile.Map(iaddr);
            
        const UINT32 size = INS_MemoryWriteSize(ins);

        const BOOL   single = (size <= 4);
                
        if( KnobTrackStores )
        {
            if( single )
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) StoreSingle,
                    IARG_MEMORYWRITE_EA,
                    IARG_UINT32, instId,
                    IARG_END);
            }
            else
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) StoreMulti,
                    IARG_MEMORYWRITE_EA,
                    IARG_MEMORYWRITE_SIZE,
                    IARG_UINT32, instId,
                    IARG_END);
            }
                
        }
        else
        {
            if( single )
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) StoreSingleFast,
                    IARG_MEMORYWRITE_EA,
                    IARG_END);
                        
            }
            else
            {
                INS_InsertPredicatedCall(
                    ins, IPOINT_BEFORE,  (AFUNPTR) StoreMultiFast,
                    IARG_MEMORYWRITE_EA,
                    IARG_MEMORYWRITE_SIZE,
                    IARG_END);
            }
        }
            
    }
}

/* ===================================================================== */
static void OriginalFini(int code, VOID *v)
{
    // print D-cache profile
    // @todo what does this print
    
    outFile << "PIN:MEMLATENCIES 1.0. 0x0\n";

    outFile << "HammerBlade Cache Parameters" << "\n";
    outFile << "# Reset Count: "
            << dl1_reset_count() << "\n";    
    outFile << "# Cache Size: "
	    << (dl1->CacheSize()/KILO) << "K\n";    
    outFile << "# Cache Block Size: "
	    <<  dl1->LineSize() << "\n";
    outFile << "# Associativity: "
	    << dl1->Associativity() << "\n";

    outFile << "\n";
    
    outFile << "Host CPU Cache Parameters" << "\n";
    outFile << "# Cache Size: "
	    << (dl1_intel->CacheSize()/KILO) << "K\n";    
    outFile << "# Cache Block Size: "
	    <<  dl1_intel->LineSize() << "\n";
    outFile << "# Associativity: "
	    << dl1_intel->Associativity() << "\n";

    outFile <<
        "#\n"
        "# DCACHE stats\n"
        "#\n";
    
    outFile << dl1->StatsLong("# ", CACHE_BASE::CACHE_TYPE_DCACHE);
    outFile << "\n";
    outFile << dl1_intel->StatsLong("# ", CACHE_BASE::CACHE_TYPE_DCACHE);
    
    if( KnobTrackLoads || KnobTrackStores ) {
        outFile <<
            "#\n"
            "# LOAD stats\n"
            "#\n";
        
        outFile << profile.StringLong();
    }

    
    //outFile.close();
}

static void HBPintoolFini(int code, void *v)
{
    // cacheline size => watts
    
    /* output our number */
    /* which is ? */
    /* I guess its the sum of cache hits * W1 */
#define WATTS(access_size, power)		\
    CLSIZE_WATTS_PAIR(access_size, power),
    
    static CLSIZE_WATTS_PAIR power_pairs [] = {
#include "hbm_power.inc"
    };
#undef WATTS

#define NANOSECONDS(access_size, ns)			\
    CLSIZE_LATENCY_PAIR(access_size, ns),
    
    static CLSIZE_LATENCY_PAIR latency_pairs [] = {
#include "hbm_latency.inc"
    };
#undef NANOSECONDS
    
    POWER_MAP power (power_pairs, power_pairs + array_size(power_pairs));
    LATENCY_MAP latency(latency_pairs, latency_pairs + array_size(latency_pairs));   
    UINT32 hb_ld_misses = 0,  hb_st_misses = 0;
    const std::string energy_message_prefix = "HammerBlade Energy Cost: ";
    
    for (DL1_ARRAY::iterator it = hammerblade_dl1s.begin();
	 it != hammerblade_dl1s.end();
	 it++) {
	DL1::CACHE_HAMMERBLADE *cache = *it;
	hb_ld_misses += cache->Misses(CACHE_BASE::ACCESS_TYPE_LOAD);
	hb_st_misses += cache->Misses(CACHE_BASE::ACCESS_TYPE_STORE);
    }

    // calculate approximate energy for our system
    // total misses * latency[clsize] * watts[clsize]
    UINT32 line_size = dl1->LineSize();
    ASSERTX((line_size % 32) == 0);

    POWER_MAP::iterator pwit = power.find(std::max(dl1->LineSize(), 1024u));
    if (pwit == power.end()) {
	outFile << energy_message_prefix << ": Could not find wattage for cache line size of " << dl1->LineSize() << "\n";
	return;
    }
    
    LATENCY_MAP::iterator latit = latency.find(dl1->LineSize());
    if (latit == latency.end()) {
	outFile << energy_message_prefix << ": Could not find memory access latency for cache line size of " << dl1->LineSize() << "\n";
	return;
    }
    
    double watts = pwit->second, ns = latit->second;

    // the 1e-9 factor is to account for latency being measured in nanoseconds
    outFile << "HammerBlade Energy Cost: " <<  std::scientific << watts * ns * (hb_ld_misses + hb_st_misses) * 1e-9 << " J\n";
    outFile << "Intel Energy Cost:       " <<  std::scientific << watts * ns * (dl1_intel->Misses(CACHE_BASE::ACCESS_TYPE_LOAD) * dl1_intel->Misses(CACHE_BASE::ACCESS_TYPE_STORE)) * 1e-9 << " J\n";
    //outFile.close();
}

VOID Fini(int code, VOID * v)
{
    OriginalFini(code, v);
    HBPintoolFini(code, v);
    outFile.close();
}
/* ===================================================================== */
/* Main                                                                  */
/* ===================================================================== */

static void resetDl1(void)
{    
    dl1 = new DL1::CACHE_HAMMERBLADE("HammerBlade L1",
				     KnobCacheSize.Value() * KILO,
				     KnobLineSize.Value(),
				     KnobAssociativity.Value());
    hammerblade_dl1s.push_back(dl1);
}

static void Routine(RTN rtn, void *v)
{
    if (!filter.SelectRtn(rtn))
	return;

    // reset dl1 every time the epoch marker is called
    if (RTN_Name(rtn) == KnobRtnEpochMarker.Value())
	resetDl1();
}

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
    
    resetDl1();   

    RTN_AddInstrumentFunction(Routine, NULL);
    
    profile.SetKeyName("iaddr          ");
    profile.SetCounterName("dcache:miss        dcache:hit");

    COUNTER_HIT_MISS threshold;

    threshold[COUNTER_HIT] = KnobThresholdHit.Value();
    threshold[COUNTER_MISS] = KnobThresholdMiss.Value();
    
    profile.SetThreshold( threshold );

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
