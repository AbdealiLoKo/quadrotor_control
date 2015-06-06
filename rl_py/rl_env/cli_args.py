import argparse

parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="Provides a set of reinforcement learning environments"
                "(gridworlds, mountain car, cart pole, stock trading, robot"
                "car simulations) for RL agents to learn in.")

parser.add_argument('--type',
                    '-t',
                    required=True,
                    metavar="ENUM",
                    choices="taxi tworooms fourrooms energy fuelworld mcar"
                            "cartpole car2to7 car7to2 carrandom stocks"
                            "lightworld".split(),
                    help="Integer seed for random number generator")

parser.add_argument('--seed',
                    default=1,
                    help="Integer seed for random number generator")

parser.add_argument('--deterministic',
                    help="Use deterministic version of the domain."
                         "Default is stochastic",
                    action='store_true')

parser.add_argument('--delay',
                    default=0,
                    help="# steps of action delay (for mcar and tworooms)")

parser.add_argument('--lag',
                    action='store_false',
                    help="Turn on brake lag for car driving domain")

parser.add_argument('--highvar',
                    action='store_false',
                    help="Have variation fuel costs in Fuel World")

parser.add_argument('--nsectors',
                    default=3,
                    help="# sectors for stocks domain")

parser.add_argument('--nstocks',
                    default=3,
                    help="# stocks for stocks domain")

parser.add_argument('--debug',
                    help="Turn on debug printing of actions/rewards",
                    action='store_true')
