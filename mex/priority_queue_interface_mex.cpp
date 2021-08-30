#include "mex.hpp"
#include "mexAdapter.hpp"

#include <cmath>
#include <memory>
#include <queue>
#include <tuple>
#include <vector>

using matlab::mex::ArgumentList;
using std::shared_ptr;
using std::size_t;
using std::vector;

#define ID (0)
#define PRIO_VAL (1)


typedef std::tuple<size_t, double> queue_entry;


// Custom comparator for queue_entry
struct queue_entry_comparator
{
    inline bool operator() (const queue_entry &a, const queue_entry &b)
    {
        return (std::get<PRIO_VAL>(a) > std::get<PRIO_VAL>(b));
    }
};

typedef std::priority_queue<queue_entry, std::vector<queue_entry>, queue_entry_comparator> prio_q;

enum COMMAND {
    NEW,
    DELETE,
    PUSH,
    POP,
    TOP,
    SIZE
};
class MexFunction : public matlab::mex::Function {
public:
    inline void operator()(ArgumentList outputs, ArgumentList inputs) {
        const int cmd = inputs[0][0];

        if (cmd == NEW)
        {
            objects.push_back(shared_ptr<prio_q>(new prio_q));
            outputs[0] = factory.createScalar<size_t>(objects.size()-1);
            return;
        }
        const size_t iObj = inputs[1][0];
        shared_ptr<prio_q> pq = objects[iObj];

        if (cmd == DELETE)
        {
            pq.reset();
            return;
        }


        if (cmd == PUSH)
        {
            size_t id = inputs[2][0];
            double prio_val = inputs[3][0];
            pq->push(queue_entry(id, prio_val));
            return;
        }

        if (cmd == POP)
        {
            pq->pop();
            return;
        }
        if (cmd == TOP)
        {
            queue_entry qe = pq->top();
            outputs[0] = factory.createScalar<size_t>(std::get<ID>(qe));
            if (outputs.size()>1)
            {
                outputs[1] = factory.createScalar<double>(std::get<PRIO_VAL>(qe));
            }
            return;
        }
        if (cmd == SIZE)
        {
            outputs[0] = factory.createScalar<size_t>(pq->size());
            return;
        }
    }

private:
    vector<shared_ptr<prio_q>> objects;
    // Create MATLAB data array factory
    matlab::data::ArrayFactory factory;
};