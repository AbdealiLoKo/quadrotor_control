
# Models
MODELS = {
    "RMAX" : 0,
    "TABULAR" : "Tabular",
    "SLF" : "SLF",
    "C45TREE" : "C4.5 Tree",
    "SINGLETREE" : "Single Tree",
    "SVM" : "SVM",
    "STUMP" : "Stump",
    "M5MULTI" : "M5 Tree",
    "M5SINGLE" : "M5 Tree",
    "M5ALLMULTI" : "M5 Tree",
    "M5ALLSINGLE" : "M5 Tree",
    "LSTMULTI" : "LS Tree",
    "LSSINGLE" : "LS Tree",
    "ALLM5TYPES" : "M5 Combo",
    "GPREGRESS" : "GP Regression",
    "GPTREE" : "GP Tree"
}

# Model combos
MODE_COMBOS = {
    "AVERAGE" : "Average",
    "WEIGHTAVG" : "Weighted Average",
    "BEST" : "Best",
    "SEPARATE" : "Separate" # Sep model for planning and forest for uncertainity
}

# Exploration
EXPLORATION = {
    "EXPLORE_UNKNOWN" : "Explore Unknowns",
    "TWO_MODE" : "Two Models",
    "TWO_MODE_PLUS_R" : "Two Models +R",
    "CONTINUOUS_BONUS" : "Continuous Bonus",
    "THRESHOLD_BONUS" : "Threshold Bonus",
    "CONTINUOUS_BONUS_R" : "Continuous Bonus +R",
    "THRESHOLD_BONUS_R" : "Threshold Bonur +R",
    "NO_EXPLORE" : "Greedy",
    "GREEDY" : "Greedy",
    "EPSILONGREEDY" : "Epsilon-Greedy",
    "VISITS_CONF" : "Visits Confidence",
    # 10 : "Type 10",
    "UNVISITED_BONUS" : "Unvisited State Bonus",
    # 12: "Type 12",
    "UNVISITED_ACT_BONUS" : "Unvisited Action Bonus",
    # 14 : "Type 14",
    # 15 : "Type 15",
    "DIFF_AND_VISIT_BONUS" : "Model Diff & Visit Bonus",
    # 17 : "Type 17",
    "NOVEL_STATE_BONUS" : "FeatDist Bonus",
    "DIFF_AND_NOVEL_BONUS" : "Model Diff & FeatDist Bonus"
}

# Planners
PLANNERS = {
    "VALUE_ITERATION"     : "Value Iteration",
    "POLICY_ITERATION"    : "Policy Iteration",
    "PRI_SWEEPING"        : "Prioritized Sweeping",
    "UCT"                 : "UCT",
    "ET_UCT"              : "UCT",
    "ET_UCT_WITH_ENV"     : "UCT",
    "SWEEPING_UCT_HYBRID" : "Sweeping UCT Hybrid",
    "CMAC_PLANNER"        : "CMACs",
    "NN_PLANNER"          : "NN",
    "MOD_PRI_SWEEPING"    : "Mod. Pro Sweeping",
    "ET_UCT_L1"           : "UCT L=1",
    "UCT_WITH_L"          : "UCT L",
    "UCT_WITH_ENV"        : "UCT Env",
    "PARALLEL_ET_UCT"     : "Parallel UCT",
    "ET_UCT_ACTUAL"       : "Real-Valued UCT",
    "ET_UCT_CORNERS"      : "Corner UCT",
    "PAR_ETUCT_ACTUAL"    : "Parallel Real-Valued UCT",
    "PAR_ETUCT_CORNERS"   : "Parallel Corner UCT",
    "POMDP_ETUCT"         : "Delayed UCT",
    "POMDP_PAR_ETUCT"     : "Parallel Delayed UCT",
    "MBS_VI"              : "Model Based Simulation - VI"
}
