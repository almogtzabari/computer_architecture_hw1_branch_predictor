/* 046267 Computer Architecture - Winter 2019/20 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include <vector>
#include <assert.h>
#include "bp_api.h"
#include "stdlib.h"
#include "math.h"
#define TARGET_SIZE 30
namespace helpers {
    int log(int num) {
        int ret_val = 0;
        while (num > 1) {
            ret_val++;
            num /= 2;
        }
        return ret_val;
    }

    uint32_t extarctLastNBits(uint32_t num, uint32_t n) {
        int mask = 0;
        for (int i = 0; i < n; i++) {
            mask +=(int)pow(2, i);
        }
        return num & mask;
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
                break;
            case WEAKLY_NOT_TAKEN:
                state = WEAKLY_TAKEN;
                break;
            case WEAKLY_TAKEN:
                state = STRONGLY_TAKEN;
                break;
            case STRONGLY_TAKEN:
                state = STRONGLY_TAKEN;
                break;
        }
    }

    /**
     * Decreasing the state of the machine.
     */
    void decreaseState() {
        switch (state) {
            case STRONGLY_NOT_TAKEN:
                state = STRONGLY_NOT_TAKEN;
                break;
            case WEAKLY_NOT_TAKEN:
                state = STRONGLY_NOT_TAKEN;
                break;
            case WEAKLY_TAKEN:
                state = WEAKLY_NOT_TAKEN;
                break;
            case STRONGLY_TAKEN:
                state = WEAKLY_TAKEN;
                break;
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

        bool compareTag(int tag) {
            return this->valid && tag == this->tag;
        }

        void setTarget(int target) {
            this->target = target;
        }

        void updateHistory (bool correct_prediction, uint32_t n_bits) {
            *history = *history<<1;
            *history+=correct_prediction;
            *history=helpers::extarctLastNBits(*history, n_bits);
        }

        bool isValid(){
            return this->valid;
        }

        void setValid(){
            this->valid=true;
        }

        StateMachineTablePtr getStateMachineTablePtr(){
            return machine_table_ptr;
        }

        void setStateMachineTablePtr(StateMachineTablePtr table){
            this->machine_table_ptr=table;
        }

        uint32_t* getHistoryPtr(){
            return this->history;
        }

        void setHistoryPtr(uint32_t* ptr){
            this->history=ptr;
        }

        uint32_t getTarget(){
            return this->target;
        }

        void setTag(uint32_t tag){
            this->tag=tag;
        }

    };

private:
    Statistics stats;
    std::vector <BBMRecord> records;
    uint32_t* ghr_ptr;
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
            case 1: return helpers::extarctLastNBits(pc>>2,history_size); // Using share with lsb.
            case 2: return helpers::extarctLastNBits(pc>>15,history_size); // Using share with mid.
            default: return 0;
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

    void computeMemorySize(){
        unsigned btb_size=records.size();
        if(!global_fsm_table_ptr){
            if(!ghr_ptr){
                // Local history, local fsm table.
                stats.size = btb_size* (unsigned)(tag_size+ TARGET_SIZE +(unsigned)2*pow(2,history_size) + history_size );
            }
            else {
                // Global history, local fsm table.
                stats.size = btb_size* (unsigned)(tag_size+ TARGET_SIZE +(unsigned)2*pow(2,history_size))+history_size;
            }
        }
        else{
            if(!ghr_ptr){
                // Local history, global fsm table.
                stats.size = btb_size* (unsigned)(tag_size+ TARGET_SIZE + history_size )+(unsigned)2*pow(2,history_size);
            }
            else {
                // Global history, global fsm table.
                stats.size = btb_size* (unsigned)(tag_size+ TARGET_SIZE)+(unsigned)2*pow(2,history_size)+ history_size;
            }

        }
    }
    BimodialStateMachine& getMachineByPC(uint32_t pc){
        assert(branchExists(pc));
        uint32_t index = getIndexByPC(pc);
        BBMRecord& record = records[index];
        StateMachineTablePtr temp = (record.getStateMachineTablePtr());
        uint32_t machine_index = *(record.getHistoryPtr()) ^ getMask(pc);
        return (*temp)[machine_index];
    }

    void insertBranchToEmptyLine(uint32_t pc, uint32_t targetPc){
        assert(!branchExists(pc));

        // Get BTB line
        uint32_t index = getIndexByPC(pc);
        BBMRecord& record = records[index];

        assert(!record.isValid());
        record.setValid();

        // Set tag
        uint32_t tag = helpers::extarctLastNBits(pc>>2, this->tag_size);
        record.setTag(tag);

        record.setTarget(targetPc);

        // Set history register
        (!this->ghr_ptr) ? record.setHistoryPtr(new uint32_t(0)) : record.setHistoryPtr(this->ghr_ptr);

        // Set state machine table
        if(!this->global_fsm_table_ptr){
            record.setStateMachineTablePtr(new StateMachineTable((unsigned)pow(2,(double)history_size), BimodialStateMachine((BimodialState)fsm_default_state)));
        }
        else{
            record.setStateMachineTablePtr(this->global_fsm_table_ptr);
        }
    }

    void insertBranchToExistingLine(uint32_t pc, uint32_t targetPc){
        assert(!branchExists(pc));

        // Get BTB line
        uint32_t index = getIndexByPC(pc);
        BBMRecord& record = records[index];
        assert(record.isValid());

        // Set branch tag in BTB
        uint32_t tag = helpers::extarctLastNBits(pc>>2, this->tag_size);
        record.setTag(tag);

        record.setTarget(targetPc);

        if(!this->ghr_ptr){
            *record.getHistoryPtr()= 0;
        }
        if(!this->global_fsm_table_ptr){
            delete record.getStateMachineTablePtr();
            record.setStateMachineTablePtr(new StateMachineTable((unsigned)pow(2,(double)history_size), BimodialStateMachine((BimodialState)fsm_default_state)));
        }
    }

public:
    BimodialBranchPredictor(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
                            bool isGlobalHist, bool isGlobalTable, int Shared) : stats(Statistics{0, 0, 0}),
                                                                                 records(std::vector<BBMRecord>(
                                                                                         btbSize)),
                                                                                 ghr_ptr(nullptr), global_fsm_table_ptr(nullptr),
                                                                                 tag_size(tagSize),
                                                                                 fsm_default_state(fsmState), shared(Shared),history_size(historySize){
        if (isGlobalHist) {
            this->ghr_ptr = new uint32_t(0);
        }
        if(isGlobalTable){
            this->global_fsm_table_ptr = new StateMachineTable(pow(2,historySize), BimodialStateMachine(BimodialState((fsmState))));
            shared = Shared;
        }
        this->computeMemorySize();
    }

    ~BimodialBranchPredictor() {
        for(BBMRecord& record: this->records){
            if(record.isValid()){
                if(!ghr_ptr) {
                    delete record.getHistoryPtr();
                }
                if(!global_fsm_table_ptr){
                    delete record.getStateMachineTablePtr();
                }
            }
        }
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
        uint32_t machine_index = *(record.getHistoryPtr()) ^ getMask(pc);
        StateMachineTablePtr temp = (record.getStateMachineTablePtr());
        StateMachineTable temp2 = *temp;
        bool prediction = (temp2[machine_index].getState() > 1);
        if(prediction){
            *dst = record.getTarget();
        }
        return prediction;
    }

    void update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
        this->stats.br_num++;
        uint32_t index = getIndexByPC(pc);
        BBMRecord& record = records[index];
        if(branchExists(pc)){
            // Branch exists in the BTB
            record.setTarget(targetPc);
            BimodialStateMachine& machine = getMachineByPC(pc);
            stats.flush_num += ((taken != (machine.getState() > 1) || (taken && (targetPc != pred_dst))));
            taken ? machine.increaseState() : machine.decreaseState();
            record.updateHistory(taken, this->history_size);
        }
        else{
            // Branch does not exist in the BTB
            stats.flush_num += taken;
            if (!record.isValid()){
                this->insertBranchToEmptyLine(pc, targetPc);

                // Update machine state
                BimodialStateMachine& machine = getMachineByPC(pc);
                taken ? machine.increaseState() : machine.decreaseState();

                // Update history
                record.updateHistory(taken, this->history_size);
            }
            else{
                this->insertBranchToExistingLine(pc, targetPc);
                BimodialStateMachine machine = this->getMachineByPC(pc);
                taken ? machine.increaseState() : machine.decreaseState();
                record.updateHistory(taken, this->history_size);
            }
        }
    }

    Statistics getStatistics(){
        return this->stats;
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
    return predictor->predict(pc, dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    predictor->update(pc, targetPc, taken, pred_dst);
}

void BP_GetStats(SIM_stats *curStats) {
    *curStats=predictor->getStatistics();
}

