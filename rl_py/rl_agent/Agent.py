
# Experience <s,a,s',r> struct
# struct experience {
#   std::vector<float> s;
#   int act;
#   float reward;
#   std::vector<float> next;
#   bool terminal;
# };

# /** Training instances for prediction models */
# struct modelPair {
#   std::vector<float> in;
#   std::vector<float> out;
# };

# /** Training instances for classification models */
# struct classPair {
#   std::vector<float> in;
#   float out;
# };

# /** Interface for an environment, whose states can be represented as
#     vectors of floats and whose actions can be represented as ints.
#     Implementations of the Environment interface determine how actions
#     influence sensations.  Note that this design assumes only one
#     agent: it would be more accurate to name this interface
#     EnvironmentAsPerceivedByOneParticularAgent. */
# class Environment {
# public:
#   /** Provides access to the current sensation that the environment
#       gives to the agent.
#       \return The current sensation. */
#   virtual const std::vector<float> &sensation() const = 0;

#   /** Allows an agent to affect its environment.
#       \param action The action the agent wishes to apply.
#       \return The immediate one-step reward caused by the action. */
#   virtual float apply(int action) = 0;

#   /** Determines whether the environment has reached a terminal state.
#       \return true iff the task is episodic and the present episode
#       has ended.  Nonepisodic tasks should simply always
#       return false. */
#   virtual bool terminal() const = 0;

#   /** Resets the internal state of the environment according to some
#       initial state distribution.  Typically the user calls this only
#       for episodic tasks that have reached terminal states, but this
#       usage is not required. */
#   virtual void reset() = 0;

#   /** Returns the number of actions available in this environment.
#       \return The number of actions available */
#   virtual int getNumActions() = 0;

#   /** Gets the minimum and maximum of the features in the environment.
#    */
#   virtual void getMinMaxFeatures(std::vector<float> *minFeat,
#                                  std::vector<float> *maxFeat) = 0;

#   /** Gets the minimum and maximum one-step reward in the domain. */
#   virtual void getMinMaxReward(float *minR, float *maxR) = 0;

#   /** Returns if the domain is episodic (true by default). */
#   virtual bool isEpisodic(){ return true; };

#   /** Get seeding experiences for agent. */
#   virtual std::vector<experience> getSeedings()
#   {
#     std::vector<experience> e;
#     return e;
#   } ;

#   /** Set the current state for testing purposes. */
#   virtual void setSensation(std::vector<float> s){};

#   virtual ~Environment() {};

# };

class Agent:
    """
    Interface for an agent. Implementations of the Agent interface
    determine the choice of actions given previous sensations and
    rewards.
    """
    def first_action(self, sensation):
        """
        Determines the first action that an agent takes in an
        environment.  This method implies that the environment is
        currently in an initial state.

        :param sensation: The initial sensation from the environment.
        :return:          The action the agent wishes to take first.
        """
        raise NotImplementedError()

    def next_action(self, reward, sensation):
        """
        Determines the next action that an agent takes in an environment
        and gives feedback for the previous action.  This method may
        only be called if the last method called was first_action or
        next_action.

        :param reward:    The one-step reward resulting from the previous action.
        :param sensation: The current sensation from the environment.
        :return:          The action the agent wishes to take next.
        """
        raise NotImplementedError()

    def last_action(self, reward):
        """
        Gives feedback for the last action taken. This method may only
        be called if the last method called was first_action or
        next_action. It implies that the task is episodic and has just
        terminated. Note that terminal sensations (states) are not
        represented.

        :param reward: The one-step reward resulting from the previous action.
        """
        pass

    def set_debug(self, debug):
        """
        Set some debug flags on/off
        """
        pass

    def seed_exp(self, seeds):
        """
        Use the model seeds from the environment to initialize the agent or
        its model
        """
        raise NotImplementedError()

    def save_policy(self):
        """
        Save the current policy to a file
        """
        raise NotImplementedError()

# /** Interface for a model that predicts a vector of floats given a vector of floats as input. */
# class Model {
# public:
#   /** Train the model on a vector of training instances */
#   virtual bool trainInstances(std::vector<modelPair> &instances) = 0;

#   /** Train the model on a single training instance */
#   virtual bool trainInstance(modelPair &instance) = 0;

#   /** Get the model's prediction for a given input */
#   virtual std::vector<float> testInstance(const std::vector<float> &in) = 0;

#   virtual ~Model() {};
# };

# /** Interface for a classification model that predicts a class given a vector of floats as input. */
# class Classifier {
# public:
#   /** Train the model on a vector of training instances */
#   virtual bool trainInstances(std::vector<classPair> &instances) = 0;

#   /** Train the model on a single training instance */
#   virtual bool trainInstance(classPair &instance) = 0;

#   /** Get the model's prediction for a given input */
#   virtual void testInstance(const std::vector<float> &in, std::map<float, float>* retval) = 0;

#   /** Get the model's confidence in its predictions for a given input. */
#   virtual float getConf(const std::vector<float> &in) = 0;

#   /** Get a copy of the model */
#   virtual Classifier* getCopy() = 0;

#   virtual ~Classifier() {};
# };

# /** All the relevant information predicted by a model for a given state-action.
#     This includes predicted reward, next state probabilities, probability of episod termination, and model confidence.
# */
# struct StateActionInfo {
#   bool known;
#   float reward;
#   float termProb;
#   int frameUpdated;

#   // map from outcome state to probability
#   std::map< std::vector<float> , float> transitionProbs;

#   StateActionInfo(){
#     known = false;
#     reward = 0.0;
#     termProb = 0.0;
#     frameUpdated = -1;
#   };
# };


# /** Interface for a model of an MDP. */
# class MDPModel {
# public:
#   /** Update the MDP model with a vector of experiences. */
#   virtual bool updateWithExperiences(std::vector<experience> &instances) = 0;

#   /** Update the MDP model with a single experience. */
#   virtual bool updateWithExperience(experience &instance) = 0;

#   /** Get the predictions of the MDP model for a given state action */
#   virtual float getStateActionInfo(const std::vector<float> &state, int action, StateActionInfo* retval) = 0;

#   /** Get a copy of the MDP Model */
#   virtual MDPModel* getCopy() = 0;
#   virtual ~MDPModel() {};
# };

# /** Interface for planners */
# class Planner {
# public:
#   /** Give the planner the model being used with the agent */
#   virtual void setModel(MDPModel* model) = 0;

#   /** Update the given model with an experience <s,a,s',r>. */
#   virtual bool updateModelWithExperience(const std::vector<float>& last,
#                                          int act,
#                                          const std::vector<float>& curr,
#                                          float reward, bool terminal) = 0;

#   /** Plan a new policy suing the current model. */
#   virtual void planOnNewModel() = 0;

#   /** Return the best action for a given state. */
#   virtual int getBestAction(const std::vector<float> &s) = 0;

#   /** Save the policy to a file. */
#   virtual void savePolicy(const char* filename) {};

#   /** Set whether the next experiences are seeds or actual experiences from the agent. */
#   virtual void setSeeding(bool seeding) {};

#   /** Set if this is the first experience of the agent. */
#   virtual void setFirst() {};

#   /** A method to return at random one of the maximum values in the vector. 
#       Such that when more than one actions are optimal, we select one of them at random.
#   */
#   std::vector<float>::iterator
#   random_max_element(std::vector<float>::iterator start,
#            std::vector<float>::iterator end) {
#     const float Q_EPSILON = 1e-4;
    
#     std::vector<float>::iterator max =
#     std::max_element(start, end);

#     // find # within epsilon of max
#     int nfound = 0;
#     for (std::vector<float>::iterator it = start; it != end; it++){
#       if (fabs(*it - *max) < Q_EPSILON){
#         nfound++;
#       }
#     }
    
#     // only 1: take it
#     if (nfound == 1)
#       return max;

#     // take one of close to max at random
#     for (std::vector<float>::iterator it = start; it != end; it++){
#       if (fabs(*it - *max) < Q_EPSILON){
#         if (rng.uniform() < (1.0 / (float)nfound)){
#           return it;
#         }
#         nfound--;
#       }
#     }
    
#     return max;
#   };

#   virtual ~Planner() {};
  
#   Random rng;

# };


# #endif
