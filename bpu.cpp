//Nakas Konstantinos-Panagiotis 2501

/*! @file
 *  This is a Branch Target Buffer (BTB) Simulator using the PIN tool
 */

#include "pin.H"
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <deque>
#include <math.h>

// --------------------------
// Replace with your own predictor
//#include "StaticPredictor.h"
//StaticPredictor myBPU; //(2048, 4);
// --------------------------

/* ===================================================================== */
// Command line switches
/* ===================================================================== */
KNOB<string> KnobOutputFile(KNOB_MODE_WRITEONCE, "pintool",
    "o", "btb.out", "specify output file name for BTB simulator");

KNOB<UINT32> KnobMispredRate(KNOB_MODE_WRITEONCE, "pintool",
    "mpr", "20", "specify direction misprediction rate for BTB simulator");

// Add your KNOBs here
KNOB<int> KnobBTBSize(KNOB_MODE_WRITEONCE, "pintool",
    "btb","64", "BTB number of entries");

KNOB<int> KnobAssociativity(KNOB_MODE_WRITEONCE, "pintool",
    "a","4", "BTB associativity");

KNOB<int> KnobRASSize(KNOB_MODE_WRITEONCE, "pintool",
    "ras","16", "RAS number of entries");

KNOB<int> KnobTagSize(KNOB_MODE_WRITEONCE, "pintool",
    "tag","48", "Size of tag");

/* ===================================================================== */
// Branch Target Buffer object & simulation methods
/* ===================================================================== */




class BPU {
public:
//BPU();  // Make your own constructor!

BPU();

bool PredictDirection(ADDRINT PC,
                      bool isControlFlow,
                      bool brTaken);

ADDRINT PredictTarget(ADDRINT PC,
                      ADDRINT fallThroughAddr,
                      bool predictDir);

VOID UpdatePredictor(ADDRINT PC,         // address of instruction executing now
                     bool brTaken,       // the actual direction
                     ADDRINT targetPC,   // the next PC, **if taken**
                     ADDRINT returnAddr, // return address for subroutine calls,
                     bool isCall,        // is a subroutine call
                     bool isReturn,      // is a return from subroutine
                     bool correctDir,    // my direction prediction was correct
                     bool correctTarg);  // my target prediction was correct

std::string ReportCounters();

private:

typedef struct information{
	ADDRINT tag;
	int valid;
	int R;
	ADDRINT next_addr;
} info;


int elements_of_stack;

info** BTB;
int* priority_table;
std::deque<ADDRINT> RAS;
}; // end class BPU

/*!
 * Predicts the direction of instruction at address PC: true - branch is taken
 *   Not a real predictor: randomly makes wrong predictions
 *   only for control flow instructions
 * This function is called for every instruction executed.
 * @param[in]   PC              address of current instruction
 * @param[in]   isControlFlow   true if instruction may change control flow
 * @param[in]   brTaken         true if instruction takes a branch
 */
bool BPU::PredictDirection(ADDRINT PC,
                           bool isControlFlow,
                           bool brTaken)
{
    if (!isControlFlow) {
        // CHEAT: can't do this in real hardware!
        // Don't make wrong predictions on normal instructions!
        // Phantom branches may appear if this is modified:
        //  normal instructions mistaken for branches because they are predicted
        //  taken and have a target address in the BTB....
        return brTaken; // should be false.
    }
    // --------------------------
    // Get a random number up to 100
    // http://stackoverflow.com/questions/822323/how-to-generate-a-random-number-in-c

    // Chop off all of the values that would cause skew...
    long end = RAND_MAX / 100; // truncate skew
    end *= 100;

    // ... and ignore results from rand() that fall above that limit.
    // (Worst case the loop condition should succeed 50% of the time,
    // so we can expect to bail out of this loop pretty quickly.)
    UINT32 r;
    while ((r = rand()) >= end)
        ;

    // -------------------------
    if (r % 100 > (100-KnobMispredRate.Value()))
        return !brTaken;  // mispredict direction
    return brTaken;
}


/*!
// Initialize data structures for branch predictors here.
// @note  Use KNOBs to pass parameters to the BPU, such as:
//   number of entries, associativity, RAS entries, replacement policies, ...
 */
//BPU BPU()
BPU::BPU(){

  elements_of_stack=0;

  BTB = new info*[KnobBTBSize.Value()/KnobAssociativity.Value()];
  priority_table = new int[KnobBTBSize.Value()/KnobAssociativity.Value()];
 	for(int i = 0; i < KnobBTBSize.Value()/KnobAssociativity.Value(); i++){
    BTB[i] = new info[KnobAssociativity.Value()];
 	}

 	for(int i = 0; i < (int) KnobBTBSize.Value()/KnobAssociativity.Value(); i++){
    priority_table[i] = 0;
 		for(int j = 0; j < (int) KnobAssociativity.Value(); j++){
 			BTB[i][j].tag=-1;
 			BTB[i][j].valid=0;
 			BTB[i][j].R=0;
 			BTB[i][j].next_addr=-1;
 		}
 	}
}
/*!
// Predict the target of the instruction at address PC by looking it up in
//  the BTB.  Use the direction prediction predictDir to decide between the
//  target address in the BTB or the fallThroughAddr
 * @param[in]   PC              address of current instruction
 * @param[in]   fallThroughAddr address of next, sequential instruction
 * @param[in]   predictDir      the predicted direction of this "branch"
 */
ADDRINT BPU::PredictTarget(ADDRINT PC,
                           ADDRINT fallThroughAddr,
                           bool predictDir)
{
  if (predictDir){
      int total_sets = KnobBTBSize.Value()/KnobAssociativity.Value();
      int line = PC & (total_sets - 1);
      int total_bits_set = log2(total_sets);
      ADDRINT tag = (PC >> (48+total_bits_set-KnobTagSize.Value()));
      for(int j = 0; j < KnobAssociativity.Value(); j++){
          if (tag == BTB[line][j].tag && BTB[line][j].valid == 1 && BTB[line][j].R == 0){
              return BTB[line][j].next_addr;
          }else if(tag == BTB[line][j].tag && BTB[line][j].valid == 1 && BTB[line][j].R == 1){
              ADDRINT ret_addr = RAS.front();
              return ret_addr;
          }
      }
  }
  return fallThroughAddr;
}

/*!
// Update the information in the BTB, RAS for the branch instruction
// at address PC, using the fully available information now that it
// has been executed.
 * @param[in]   PC           address of current instruction
 * @param[in]   brTaken      true if is actually taken
 * @param[in]   targetPC     the target, if it is taken
 * @param[in]   returnAddr   the return address, if it is a subroutine call
 * @param[in]   isCall       true if this is a subroutine call
 * @param[in]   isReturn     true if this is a subroutine return
 * @param[in]   correctDir   true if the direction was predicted correctly
 * @param[in]   correctTarg  true if the target was predicted correctly
// @note Use KNOBs to pass parameters related to BTB prediction such as
//   replacement policies, when to insert an entry, ...
*/
VOID BPU::UpdatePredictor(ADDRINT PC,           // address of instruction executing now
                          bool brTaken,      // the actual direction
                          ADDRINT targetPC,     // the next PC, **if taken**
                          ADDRINT returnAddr,   // return address for subroutine calls,
                          bool isCall,       // is a subroutine call
                          bool isReturn,     // is a return from subroutine
                          bool correctDir,   // my direction prediction was correct
                          bool correctTarg)  // my target prediction was correct
{
  int total_sets = KnobBTBSize.Value()/KnobAssociativity.Value();
  int line = PC & (total_sets - 1);
  int total_bits_set = log2(total_sets);
  ADDRINT tag = (PC >> (48+total_bits_set-KnobTagSize.Value()));
  if (isCall){
    if (elements_of_stack < KnobRASSize.Value()){
      RAS.push_front(returnAddr);
      elements_of_stack++;
    }else{
      RAS.pop_back();
      RAS.push_front(returnAddr);
    }
  }

  if (isReturn && !RAS.empty()){
  	RAS.pop_front();
  	elements_of_stack--;
  }

  for(int j = 0; j < KnobAssociativity.Value(); j++){
    if (BTB[line][j].tag==tag && BTB[line][j].valid==1){
    	BTB[line][j].next_addr = targetPC;
      return;
    }
  }
  for(int j = 0; j < KnobAssociativity.Value(); j++){
    if (BTB[line][j].valid==0){
      BTB[line][j].tag = tag;
      BTB[line][j].valid = 1;
      BTB[line][j].next_addr = targetPC;
      return;
    }
  }
  BTB[line][priority_table[line]].tag = tag;
  BTB[line][priority_table[line]].next_addr = targetPC;
  BTB[line][priority_table[line]].valid=1;
  if (priority_table[line] == KnobAssociativity.Value()-1){
    priority_table[line]=0;
  }else{
    priority_table[line]++;
  }
}


std::string BPU::ReportCounters()
{
    return std::string();
}

/* ================================================================== */
// Global variables
/* ================================================================== */
BPU  *myBPU; // The Branch Prediction Unit
std::ofstream *outFile;   // File for simulation output

// Global counters for the simulator:
static UINT64 cnt_instr = 0;
static UINT64 cnt_branches = 0;
static UINT64 cnt_branches_taken = 0;
static UINT64 cnt_correctPredDir = 0;
static UINT64 cnt_correctPredTarg = 0;
static UINT64 cnt_correctPred     = 0;


/* ===================================================================== */
// Utilities
/* ===================================================================== */

/*!
 *  Print out help message.
 */
INT32 Usage()
{
    cerr << "This tool is a Branch Targer Simulator." << endl <<
            "It runs an application and prints out the number of" << endl <<
            "dynamically executed branches, their target prediction ratio," << endl <<
            "and other metrics." << endl << endl;
    cerr << KNOB_BASE::StringKnobSummary() << endl;
    return -1;
}


/* ===================================================================== */
// Analysis routines
/* ===================================================================== */

/*!
 * Process branches: predict all instructions at Fetch, check prediction
 *  and update prediction structures at Execute stage
 * This function is called for every instruction executed.
 * @param[in]   PC              address of current instruction
 * @param[in]   targetPC        the next PC, **if taken**
 * @param[in]   brTaken         the branch direction, 0 - not taken, 1 - taken
 * @param[in]   size            the instruction size in bytes
 * @param[in]   isCall          true if the instruction is a subroutine call
 * @param[in]   isReturn        true if the instruction is a subroutine return
 * @param[in]   isConstrolFlow  true if the instruction is branch, jump, return, ...
 */
VOID ProcessBranch(ADDRINT PC,
                   ADDRINT targetPC,
                   bool brTaken,
                   UINT32 size,
                   bool isCall,
                   bool isReturn,
                   bool isControlFlow)
{
   /*
   *outFile << "PC: "          << PC
            << " targetPC: "   << targetPC
            << " taken: "      << brTaken
            << " isCall: "     << isCall
            << " isRet: "      << isReturn
            << " isBrOrCall: " << isControlFlow
            << " PC+size: " << PC+size
           << endl;
    */
    ADDRINT fallThroughAddr = PC + size;
    bool    correctDir  = false;
    bool    correctTarg = false;
    bool    predictDir;
    ADDRINT predictPC;

    // ------------------------------------------
    // Make your prediction:  (@ Fetch stage)
    predictDir = myBPU->PredictDirection(PC, isControlFlow, brTaken);
    predictPC  = myBPU->PredictTarget(PC, fallThroughAddr, predictDir);
    // ------------------------------------------


    // ------------------------------------------
    // Update counters, check prediction
    cnt_instr++;
    if (isControlFlow) {
        cnt_branches++;
        if (brTaken)
            cnt_branches_taken++;
    }

    if (predictDir == brTaken) { // Correct prediction of branch direction
        correctDir = true;
        if (isControlFlow) {
            cnt_correctPredDir++; // Count correct predictions for actual branches
        }
    }

    if (brTaken) { // brach was actually taken
        if (predictPC == targetPC) { // Target predicted
            correctTarg = true;
            if (isControlFlow) {
                cnt_correctPredTarg++;
            }
        }
    } else { // not actually taken
        if (predictPC == fallThroughAddr) {
            correctTarg = true;
            if (isControlFlow) {
                cnt_correctPredTarg++;
            }
        }
    }
    if (correctTarg && correctDir && isControlFlow)
        cnt_correctPred++;
    // ------------------------------------------

    // ------------------------------------------
    if (isControlFlow) {
        // Update the state of the predictor:  (@ execute stage only)
        myBPU->UpdatePredictor(
                PC,               // address of instruction executing now
                brTaken,          // the actual direction
                targetPC,         // the next PC, **if taken**
                fallThroughAddr,  // return address for subroutine calls,
                //     DO NOT STORE IN BTB!
                isCall,           // is a subroutine call
                isReturn,         // is a return from subroutine
                correctDir,       // my direction prediction was correct
                correctTarg       // my target prediction was correct
        );
    }
    // ------------------------------------------
}

/* ===================================================================== */
// Instrumentation callbacks
/* ===================================================================== */

/*!
 * Insert call to the analysis routine before every branch instruction
 * of the trace.
 * This function is called every time a new trace is encountered.
 * @param[in]   trace    trace to be instrumented
 * @param[in]   v        value specified by the tool in the TRACE_AddInstrumentFunction
 *                       function call
 */
VOID Instruction(INS ins, VOID *v)
{
    if (INS_IsBranchOrCall(ins)) {   // Branch or call. Includes returns
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) ProcessBranch,
                       IARG_INST_PTR,                 // The instruction address
                       IARG_BRANCH_TARGET_ADDR,       // target address of the branch, or return address
                       IARG_BRANCH_TAKEN,             // taken branch (0 - not taken. BOOL)
                       IARG_UINT32,  INS_Size(ins),   // instr. size - used to calculare return address for subroutine calls
                       IARG_BOOL, INS_IsCall(ins),    // is this a subroutine call (BOOL)
                       IARG_BOOL, INS_IsRet(ins),     // is this a subroutine return (BOOL)
                       IARG_BOOL, INS_IsBranchOrCall(ins),
                       IARG_END);
    } else {   //  not a flow-control instruction
        INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) ProcessBranch,
                       IARG_INST_PTR,                 // The instruction address
                       IARG_ADDRINT, (ADDRINT) 0,     // target address of the branch, or return address
                       IARG_BOOL, false,              // taken branch (0 - not taken. BOOL)
                       IARG_UINT32,  INS_Size(ins),   // instr. size - used to calculare return address for subroutine calls
                       IARG_BOOL, INS_IsCall(ins),    // is this a subroutine call (BOOL)
                       IARG_BOOL, INS_IsRet(ins),     // is this a subroutine return (BOOL)
                       IARG_BOOL, INS_IsBranchOrCall(ins),
                       IARG_END);
    }
}


/*!
 * Print out analysis results.
 * This function is called when the application exits.
 * @param[in]   code            exit code of the application
 * @param[in]   v               value specified by the tool in the
 *                              PIN_AddFiniFunction function call
 */
VOID Fini(INT32 code, VOID *v)
{
    // outFile is already opened in main.
    *outFile <<  "===================================================" << endl;
    *outFile <<  "This application is instrumented by BTBsim PIN tool" << endl;

    *outFile << "Instructions: " << cnt_instr << endl;
    *outFile << "Branches: " << cnt_branches << endl;
    *outFile << " taken: " << cnt_branches_taken << "(" << cnt_branches_taken*100.0/cnt_branches << "%)" << endl;
    *outFile << " Predicted (direction & target): " << cnt_correctPred << "(" << cnt_correctPred*100.0 /cnt_branches << "%)" << endl;
    *outFile << " Predicted direction: " << cnt_correctPredDir << "(" << cnt_correctPredDir*100.0 /cnt_branches << "%)" << endl;
    *outFile << " Predicted target: " << cnt_correctPredTarg << "(" << cnt_correctPredTarg*100.0 /cnt_branches << "%)" << endl;

    // -------------------------------------------
    //  Output any extra counters/statistics here
    // -------------------------------------------
    //  Report any predictor internal counters
    std::string s = myBPU->ReportCounters();
    if (!s.empty())
        *outFile << s;

    *outFile <<  "===================================================" << endl;
    outFile->close();
}

/*!
 * The main procedure of the tool.
 * This function is called when the application image is loaded but not yet started.
 * @param[in]   argc            total number of elements in the argv array
 * @param[in]   argv            array of command line arguments,
 *                              including pin -t <toolname> -- ...
 */
int main(int argc, char * argv[])
{
    // Initialize PIN library. Print help message if -h(elp) is specified
    // in the command line or the command line is invalid
    if (PIN_Init(argc, argv))
        return Usage();

    // Check and open output file for simulation results.
    // Write to a file since cout and cerr may be closed by the application
    std::string fileName = KnobOutputFile.Value();
    if (fileName.empty()) {
        cerr << "ERROR: must have an output file.";
        exit(-1);
    }
    outFile = new std::ofstream(fileName.c_str());

    myBPU = new BPU(); // Initialise Branch Prediction Unit

    // Register Instruction to be called to instrument instructions
    INS_AddInstrumentFunction(Instruction, 0);

    // Register Fini to be called when the application exits
    PIN_AddFiniFunction(Fini, 0);

    // Start the program, never returns
    PIN_StartProgram();

    return 0;
}
/* ===================================================================== */
/* eof */
/* ===================================================================== */
