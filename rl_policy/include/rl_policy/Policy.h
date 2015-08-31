#include <vector>

class Policy {
public:    
    virtual int get_n_weights();
    virtual void get_weights(std::vector<float> &weights);
    virtual void set_weights(std::vector<float> w);
    virtual void get_value(std::vector<float> &input, std::vector<float> &output);
    virtual ~Policy();
};
