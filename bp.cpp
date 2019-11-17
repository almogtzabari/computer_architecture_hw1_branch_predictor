/* 046267 Computer Architecture - Winter 2019/20 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include <vector>
#include <assert.h>
#include "bp_api.h"
#include "stdlib.h"
#include "math.h"

namespace helpers{
    int log(int num){
        int ret_val = 0;
        while(num>1){
            ret_val++;
            num /= 2;
        }
        return ret_val;
    }

    uint32_t extarctLastNBits(uint32_t num, uint32_t n){
        assert(n<9 && n>0);  // BTB size can only be 1,2,4,8,16,32.
        switch(n){
            case 1: return num & 0x1;
            case 2: return num & 0x3;
            case 3: return num & 0x7;
            case 4: return num & 0xf;
            case 5: return num & 0x1f;
            case 6: return num & 0x3f;
            case 7: return num & 0x7f;
            case 8: return num & 0xff;
            default: return 0;
        }
    }
}

class BimodialStateMachine;

typedef enum bimodial_state {
    STRONGLY_NOT_TAKEN = 0,
    WEAKLY_NOT_TAKEN = 1,
    WEAKLY_TAKEN = 2,
    STRONGLY_TAKEN = 3
} BimodialState;

typedef std::vector<BimodialStateMachine> StateMachineTable;
typedef StateMachineTable* StateMachineTablePtr;
typedef SIM_stats Statistics;

typedef struct reg_t{
    std::vector<bool> data;

    /**
     * Default constructor.
     * Register will initialized with value 0.
     * @param size - number of bits to initialize the register with.
     */
    reg_t(uint32_t size=8) : data(std::vector<bool>(size)) {
        for(int i=0; i<size; i++){
            data[i] = 0;
        }
    }

    void shiftLeft() {
        for (int i = data.size() - 1; i > 0; i--) {
            data[i] = data[i - 1];
        }
        data[0] = 0;
    }

    void shiftRight() {
        for (int i = 0; i < data.size() - 1; i++) {
            data[i] = data[i + 1];
        }
        data[data.size() - 1] = 0;
    }

    bool operator[](int index) {
        return data.at(index);
    }

    const bool operator[](int index) const {
        return data.at(index);
    }

    int getSize() {
        return data.size();
    }

    int getValue(){
        int value = 1;
        for (int i=0; i<data.size(); i++){
            value += data[i] * 2^i;
        }
        return value;
    }
} Register;

class BimodialStateMachine {
private:
    BimodialState state;

public:
    /**
     * Constructor
     * @param default_state - Default state to initialize the machine with.
     */
    BimodialStateMachine(BimodialState default_state) : state(default_state) {}

    /**
     * Increasing the state of the machine.
     */
    void increaseState() {
        switch (state) {
            case STRONGLY_NOT_TAKEN:
                state = WEAKLY_NOT_TAKEN;
            case WEAKLY_NOT_TAKEN:
                state = WEAKLY_TAKEN;
            case WEAKLY_TAKEN:
                state = STRONGLY_TAKEN;
            case STRONGLY_TAKEN:
                state = STRONGLY_TAKEN;
        }
    }

    /**
     * Decreasing the state of the machine.
     */
    void decreaseState() {
        switch (state) {
            case STRONGLY_NOT_TAKEN:
                state = STRONGLY_NOT_TAKEN;
            case WEAKLY_NOT_TAKEN:
                state = STRONGLY_NOT_TAKEN;
            case WEAKLY_TAKEN:
                state = WEAKLY_NOT_TAKEN;
            case STRONGLY_TAKEN:
                state = WEAKLY_TAKEN;
        }
    }

    /**
     * Returns the state of the machine.
     */
    BimodialState getState() {
        return state;
    }
};

class BimodialBranchPredictor {

    class BBMRecord {
    private:
        bool valid;
        uint32_t tag;
        uint32_t target;
        uint32_t* history;
        StateMachineTablePtr machine_table_ptr;

    public:
        BBMRecord() : valid(false) {}

        BBMRecord(Register tag, Register target, Register *history, StateMachineTablePtr ptr) : valid(true), tag(tag),
                                                                                                target(target),
                                                                                                history(history),
                                                                                                machine_table_ptr(
                                                                                                         ptr) {}

        bool compareTag(int tag) {
            return this->valid && tag == this->tag.getValue();
        }

        void setTarget(int target) {
            assert(this->valid);
            this->target = target;
        }

        void updateHistory(bool correct_prediction) {}

        bool isValid(){
            return this->valid;
        }

        StateMachineTablePtr getStateMachineTable(){
            return machine_table_ptr;
        }

        uint32_t* getHistory(){
            return this->history;
        }

        uint32_t getTarget(){
            return this->target;
        }

    };

private:
    Statistics stats;
    std::vector <BBMRecord> records;
    Register* ghr_ptr;
    StateMachineTablePtr global_fsm_table_ptr;
    unsigned tag_size;
    unsigned fsm_default_state;
    int shared;
    unsigned history_size;

    /**
     * @param pc - branch's pc.
     * @return True if branch with given pc exists in the predictor.
     */
    bool branchExists(uint32_t pc){
        uint32_t index = getIndexByPC(pc);
        uint32_t tag = helpers::extarctLastNBits(pc>>2, this->tag_size);

        // Check edge case when BTB size is 1
        if (records.size() == 1){
            return records[0].isValid() && records[0].compareTag(tag);
        }

        return records[index].isValid() && records[index].compareTag(tag);
    }

    uint32_t getMask(uint32_t pc){
        if (!this->global_fsm_table_ptr){
            return 0;
        }
        switch (this->shared){
            case 0: return helpers::extarctLastNBits(pc>>2,history_size); // Using G-share with lsb.
            case 1: return helpers::extarctLastNBits(pc>>2,history_size); // Using L-share with lsb.
            case 2: return helpers::extarctLastNBits(pc>>15,history_size); // Using G-share with mid.
            case 3: return helpers::extarctLastNBits(pc>>15,history_size); // Using L-share with mid.
            case 4: return 0; // Shared Table without G\L - share.
            default: return 4;
        }
    }

    uint32_t getIndexByPC(uint32_t pc){
        // Calculate how many bits need to be extracted from pc
        uint32_t btb_bits_size = helpers::log(records.size());

        // Extract bits from pc
        uint32_t shifted_pc = pc >> 2;  // getting rid of 2 `00` of pc.

        // Calculates index to btb and extracts tag from record.
        return helpers::extarctLastNBits(shifted_pc, btb_bits_size);

    }

public:
    BimodialBranchPredictor(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
                            bool isGlobalHist, bool isGlobalTable, int Shared) : stats(Statistics{0, 0, 0}),
                                                                                 records(std::vector<BBMRecord>(
                                                                                         btbSize)),
                                                                                 ghr_ptr(nullptr), global_fsm_table_ptr(nullptr),
                                                                                 tag_size(tagSize),
                                                                                 fsm_default_state(fsmState), shared(-1),history_size(historySize){
        if (isGlobalHist) {
            this->ghr_ptr = new Register(historySize);
        }
        if(isGlobalTable){
            this->global_fsm_table_ptr = new StateMachineTable(pow(2,historySize), BimodialStateMachine(BimodialState(fsmState)));
            shared = Shared;
        }
    }

    ~BimodialBranchPredictor() {
        if(ghr_ptr) {
            delete this->ghr_ptr;
        }
        if(global_fsm_table_ptr){
            delete this->global_fsm_table_ptr;
        }
    }

    bool predict(uint32_t pc, uint32_t *dst){
        *dst = pc + 4;

        if(!branchExists(pc)){
            return false;
        }

        uint32_t index = getIndexByPC(pc);
        BBMRecord& record = records[index];
        uint32_t machine_index = *(record.getHistory()) ^ getMask(pc);
        StateMachineTablePtr temp = (record.getStateMachineTable());
        StateMachineTable temp2 = *temp;
        bool prediction = (temp2[machine_index].getState() > 1);
        if(prediction){
            *dst = record.getTarget();
        }
        return prediction;
    }




};

BimodialBranchPredictor* predictor;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared) {
    try{
        predictor = new BimodialBranchPredictor(btbSize, historySize, tagSize, fsmState, isGlobalHist, isGlobalTable, Shared);
    }
    catch (...){
        return -1;
    }

    return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst) {
    bool temp = predictor->branchExists(pc);
    return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    return;
}

void BP_GetStats(SIM_stats *curStats) {
    return;
}

