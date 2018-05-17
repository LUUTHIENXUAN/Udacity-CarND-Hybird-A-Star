#include <iostream>
#include <cassert>
#include <utility>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <map>
#include <vector>
#include <malloc.h>

// Max Alignment
#if defined(_MSC_VER)
    #define MAX_ALIGN __declspec(align(16))
#else
    #define MAX_ALIGN __attribute__((aligned(16)))
#endif

// https://stackoverflow.com/questions/29931592/fastest-mutable-heap-implementation-in-c
using namespace std;

static void* max_malloc(size_t amount)
{
    #ifdef _MSC_VER
        return _aligned_malloc(amount, 16);
    #else
        void* mem = 0;
        posix_memalign(&mem, 16, amount);
        return mem;
    #endif
}

static void max_free(void* mem)
{
    #ifdef _MSC_VER
        return _aligned_free(mem);
    #else
        free(mem);
    #endif
}

// Balanced priority queue for very quick insertions and
// removals when the keys are balanced across a distributed range.
template <class Key, class Value, class KeyToIndex>
class BalancedQueue
{
public:
    enum {zone_len = 256};

    /// Creates a queue with 'n' buckets.
    explicit BalancedQueue(int n):
        num_nodes(0), num_buckets(n+1), min_bucket(n+1), buckets(static_cast<Bucket*>(max_malloc((n+1) * sizeof(Bucket)))), free_nodes(0), pools(0)
    {
        const int num_zones = num_buckets / zone_len + 1;
        zone_counts = new int[num_zones];
        for (int j=0; j < num_zones; ++j)
            zone_counts[j] = 0;

        for (int j=0; j < num_buckets; ++j)
        {
            buckets[j].num = 0;
            buckets[j].head = 0;
        }
    }

    /// Destroys the queue.
    ~BalancedQueue()
    {
        clear();
        max_free(buckets);
        while (pools)
        {
            Pool* to_free = pools;
            pools = pools->next;
            max_free(to_free);
        }
        delete[] zone_counts;
    }

    /// Makes the queue empty.
    void clear()
    {
        const int num_zones = num_buckets / zone_len + 1;
        for (int j=0; j < num_zones; ++j)
            zone_counts[j] = 0;
        for (int j=0; j < num_buckets; ++j)
        {
            while (buckets[j].head)
            {
                Node* to_free = buckets[j].head;
                buckets[j].head = buckets[j].head->next;
                node_free(to_free);
            }
            buckets[j].num = 0;
        }
        num_nodes = 0;
        min_bucket = num_buckets+1;
    }

    /// Pushes an element to the queue.
    void push(const Key& key, const Value& value)
    {
        const int index = KeyToIndex()(key);
        assert(index >= 0 && index < num_buckets && "Key is out of range!");

        Node* new_node = node_alloc();
        new (&new_node->key) Key(key);
        new (&new_node->value) Value(value);
        new_node->next = buckets[index].head;
        buckets[index].head = new_node;
        assert(new_node->key == key && new_node->value == value);
        ++num_nodes;
        ++buckets[index].num;
        ++zone_counts[index/zone_len];
        min_bucket = std::min(min_bucket, index);
    }

    /// @return size() == 0.
    bool empty() const
    {
        return num_nodes == 0;
    }

    /// @return The number of elements in the queue.
    int size() const
    {
        return num_nodes;
    }

    /// Pops the element with the minimum key from the queue.
    std::pair<Key, Value> pop()
    {
        assert(!empty() && "Queue is empty!");
        for (int j=min_bucket; j < num_buckets; ++j)
        {
            if (buckets[j].head)
            {
                Node* node = buckets[j].head;
                Node* prev_node = node;
                Node* min_node = node;
                Node* prev_min_node = 0;
                const Key* min_key = &min_node->key;
                const Value* min_val = &min_node->value;
                for (node = node->next; node; prev_node = node, node = node->next)
                {
                    if (node->key < *min_key)
                    {
                        prev_min_node = prev_node;
                        min_node = node;
                        min_key = &min_node->key;
                        min_val = &min_node->value;
                    }
                }
                std::pair<Key, Value> kv(*min_key, *min_val);
                if (min_node == buckets[j].head)
                    buckets[j].head = buckets[j].head->next;
                else
                {
                    assert(prev_min_node);
                    prev_min_node->next = min_node->next;
                }
                removed_node(j);
                node_free(min_node);
                return kv;
            }
        }
        throw std::runtime_error("Trying to pop from an empty queue.");
    }

    /// Erases an element from the middle of the queue.
    /// @return True if the element was found and removed.
    bool erase(const Key& key, const Value& value)
    {
        assert(!empty() && "Queue is empty!");
        const int index = KeyToIndex()(key);
        if (buckets[index].head)
        {
            Node* node = buckets[index].head;
            if (node_key(node) == key && node_val(node) == value)
            {
                buckets[index].head = buckets[index].head->next;
                removed_node(index);
                node_free(node);
                return true;
            }

            Node* prev_node = node;
            for (node = node->next; node; prev_node = node, node = node->next)
            {
                if (node_key(node) == key && node_val(node) == value)
                {
                    prev_node->next = node->next;
                    removed_node(index);
                    node_free(node);
                    return true;
                }
            }
        }
        return false;
    }

private:
    // Didn't bother to make it copyable -- left as an exercise.
    BalancedQueue(const BalancedQueue&);
    BalancedQueue& operator=(const BalancedQueue&);

    struct Node
    {
        Key key;
        Value value;
        Node* next;
    };
    struct Bucket
    {
        int num;
        Node* head;
    };
    struct Pool
    {
        Pool* next;
        MAX_ALIGN char buf[1];
    };
    Node* node_alloc()
    {
        if (free_nodes)
        {
            Node* node = free_nodes;
            free_nodes = free_nodes->next;
            return node;
        }

        const int pool_size = std::max(4096, static_cast<int>(sizeof(Node)));
        Pool* new_pool = static_cast<Pool*>(max_malloc(sizeof(Pool) + pool_size - 1));
        new_pool->next = pools;
        pools = new_pool;

        // Push the new pool's nodes to the free stack.
        for (int j=0; j < pool_size; j += sizeof(Node))
        {
            Node* node = reinterpret_cast<Node*>(new_pool->buf + j);
            node->next = free_nodes;
            free_nodes = node;
        }
        return node_alloc();
    }
    void node_free(Node* node)
    {
        // Destroy the key and value and push the node back to the free stack.
        node->key.~Key();
        node->value.~Value();
        node->next = free_nodes;
        free_nodes = node;
    }
    void removed_node(int bucket_index)
    {
        --num_nodes;
        --zone_counts[bucket_index/zone_len];
        if (--buckets[bucket_index].num == 0 && bucket_index == min_bucket)
        {
            // If the bucket became empty, search for next occupied minimum zone.
            const int num_zones = num_buckets / zone_len + 1;
            for (int j=bucket_index/zone_len; j < num_zones; ++j)
            {
                if (zone_counts[j] > 0)
                {
                    for (min_bucket=j*zone_len; min_bucket < num_buckets && buckets[min_bucket].num == 0; ++min_bucket) {}
                    assert(min_bucket/zone_len == j);
                    return;
                }
            }
            min_bucket = num_buckets+1;
            assert(empty());
        }
    }
    int* zone_counts;
    int num_nodes;
    int num_buckets;
    int min_bucket;
    Bucket* buckets;
    Node* free_nodes;
    Pool* pools;
};

/// Test Parameters
enum {num_keys = 1000000};
enum {buckets = 100000};

static double sys_time()
{
    return static_cast<double>(clock()) / CLOCKS_PER_SEC;
}

struct KeyToIndex
{
    int operator()(double val) const
    {
        return static_cast<int>(val * buckets);
    }
};

int main()
{
    vector<double> keys(num_keys);
    for (int j=0; j < num_keys; ++j)
        keys[j] = static_cast<double>(rand()) / RAND_MAX;

    for (int k=0; k < 5; ++k)
    {
        // Multimap
        {
            const double start_time = sys_time();
            multimap<double, int> q;
            for (int j=0; j < num_keys; ++j)
                q.insert(make_pair(keys[j], j));

            // Pop each key, modify it, and reinsert.
            for (int j=0; j < num_keys; ++j)
            {
                pair<double, int> top = *q.begin();
                q.erase(q.begin());
                top.first = static_cast<double>(rand()) / RAND_MAX;
                q.insert(top);
            }
            cout << (sys_time() - start_time) << " secs for multimap" << endl;
        }

        // Balanced Queue
        {
            const double start_time = sys_time();
            BalancedQueue<double, int, KeyToIndex> q(buckets);
            for (int j=0; j < num_keys; ++j)
                q.push(keys[j], j);

            // Pop each key, modify it, and reinsert.
            for (int j=0; j < num_keys; ++j)
            {
                pair<double, int> top = q.pop();
                top.first = static_cast<double>(rand()) / RAND_MAX;
                q.push(top.first, top.second);
            }
            cout << (sys_time() - start_time) << " secs for BalancedQueue" << endl;
        }
        cout << endl;
    }
}
