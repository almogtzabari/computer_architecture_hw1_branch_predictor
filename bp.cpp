/* 046267 Computer Architecture - Winter 2019/20 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"

typedef enum bimodial_state {
    STRONGLY_NOT_TAKEN = 0,
    WEAKLY_NOT_TAKEN = 1,
    WEAKLY_TAKEN = 2,
    STRONGLY_TAKEN = 3
} BimodialState;

typedef std::vector<BimodialStateMachine> StateMachineTable;
typedef StateMachineTable* StateMachineTablePtr;
typedef SIM_stats Statistics;

struct Register {
    std::vector<bool> data;

    /**
     * Default constructor.
     * Register will initialized with value 0.
     * @param size - number of bits to initialize the register with.
     */
    Register(uint32_t size, int value = 0) : data(std::vector<bool>(size)) {
        // Logic to convert int to bits.
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
};

class BimodialStateMachine {
private:
    BimodialState state;

    /**
     * Constructor
     * @param default_state - Default state to initialize the machine with.
     */
    BimodialStateMachine(const BimodialState &default_state) : state(default_state) {}

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

public:
    /**
     * Returns the state of the machine.
     */
    void getState() {
        return state;
    }
};

class BimodialBranchPredictor {

    class BBMRecord {
    private:
        bool valid;
        Register tag;
        Register target;
        Register *history;
        StateMachineTablePtr machines_table_ptr;

    public:
        BBMRecord() : valid(false) {}

        BBMRecord(Register tag, Register target, Register *history, StateMachineVectorPtr ptr) : valid(true), tag(tag),
                                                                                                 target(target),
                                                                                                 history(history),
                                                                                                 machines_table_ptr(
                                                                                                         ptr) {}

        bool compareTag(int tag) {
            return this->valid && tag == this->tag;
        }

        void setTarget(int target) {
            assert(this->valid);
            this->target = target;
        }

        void updateHistory(bool correct_prediction) {}

    };

private:
    Statistics stats;
    std::vector <BBMRecord> records;
    Register *ghr_ptr;
    StateMachineTablePtr global_fsm_table_ptr;
    unsigned tag_size;
    unsigned fsm_default_state;

    BimodialBranchPredictor(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
                            bool isGlobalHist, bool isGlobalTable, int Shared) : stats(Statistics{0, 0, 0}),
                                                                                 records(std::vector<BBMRecord>(
                                                                                         btbSize)),
                                                                                 ghr_ptr(nullptr), global_fsm_table_ptr(nullptr),
                                                                                 tag_size(tagSize),
                                                                                 fsm_default_state(fsmState) {
        if (isGlobalHist) {
            this->ghr_ptr = new Register(historySize);
        }
        if(isGlobalTable){
            this->global_fsm_table_ptr = new StateMachineTable()
        }
    }

    ~BimodialBranchPredictor() {
        if (global_history_register_ptr) {
            delete this->global_history_register_ptr;
        }
    }

};


int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
            bool isGlobalHist, bool isGlobalTable, int Shared) {
    return -1;
}

bool BP_predict(uint32_t pc, uint32_t *dst) {
    return false;
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
    return;
}

void BP_GetStats(SIM_stats *curStats) {
    return;
}

