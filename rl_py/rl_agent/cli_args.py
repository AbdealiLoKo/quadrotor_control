import argparse

parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="A set of reinforcement learning agents to learn various tasks. "
                "In addition, a framework for model based agents where any "
                "model learning and/or planning methods can be substituted in.")

parser.add_argument('--type',
                    '-t',
                    required=True,
                    metavar="ENUM",
                    choices="qlearner sarsa modelbased rmax texplore dyna "
                            "savedpolicy".split(),
                    help="Type of agent to use")

parser.add_argument('--seed',
                    default=1,
                    help="Integer seed for random number generator")

parser.add_argument('--gamma',
                    default=0.99,
                    help="Discount factor between 0 and 1")

parser.add_argument('--epsilon',
                    default=0.1,
                    help="Epsilon for epsilon-greedy exploration")

parser.add_argument('--alpha',
                    default=0.3,
                    help="Learning rate alpha")

parser.add_argument('--initialvalue',
                    default=0.0,
                    help="Initial Q values")

parser.add_argument('--actrate',
                    default=10.0,
                    help="Action selection rate (Hz)")

parser.add_argument('--_lambda',
                    default=0.1,
                    help="Lambda for eligibility traces")

parser.add_argument('--m',
                    default=5,
                    help="Parameter for R-Max")

parser.add_argument('--k',
                    default=1000,
                    help="For Dyna: # of model based updates to do between "
                         "each real world update")

parser.add_argument('--history',
                    default=0,
                    help="# steps of history to use for planning with delay")

parser.add_argument('--filename',
                    default=None,
                    help="File to load saved policy from for savedpolicy agent")

parser.add_argument('--model',
                    default=None,
                    metavar="ENUM",
                    choices="tabular tree m5tree texplore c45tree rmax".split(),
                    help="Model type to use")

parser.add_argument('--planner',
                    default=None,
                    metavar="ENUM",
                    choices="vi pi sweeping uct parallel-uct delayed-uct "
                            "delayed-parallel-uct".split(),
                    help="Planner type to use")

parser.add_argument('--exploration',
                    default=None,
                    metavar="ENUM",
                    choices="unknown greedy epsilongreedy "
                            "variancenovelty".split(),
                    help="Exploration type to use")

parser.add_argument('--combo',
                    default=None,
                    metavar="ENUM",
                    choices="average best separate".split(),
                    help="Model combo type")

parser.add_argument('--nmodel',
                    default=1,
                    help="# of models")

parser.add_argument('--nstates',
                    default=0,
                    help="Optionally discretize domain into value # of states "
                         "on each feature")

parser.add_argument('--abstrans',
                    default=0,
                    action='store_true',
                    help="Learn relative transition")

parser.add_argument('--v',
                    default=0,
                    help="For TEXPLORE: b/v coefficient for rewarding "
                         "state-actions where models disagree")

parser.add_argument('--n',
                    default=0,
                    help="For TEXPLORE: n coefficient for rewarding "
                         "state-actions which are novel")

parser.add_argument('--debug',
                    help="Turn on debug printing of actions/rewards",
                    action='store_true')
