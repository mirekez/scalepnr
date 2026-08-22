#pragma once

#include <vector>
#include <algorithm>
#include <cstddef>
#include <limits>
#include <print>

template <class T> class Ref;
template <class T> class Referable;

template <class T>
struct RefBase
{
    T* peer = nullptr;
    size_t peer_index = std::numeric_limits<size_t>::max();
};

//extern size_t allocated;

template <class T>
struct Referable: public T
{
    std::vector<RefBase<Referable<T>>*> peers;

    Referable() {
    }
    Referable(T&& in) : T(std::move(in)) {
    }
    Referable(Referable&& in) : T(std::move(in)) {
//    printf("obj %p move from %p\n", this, &in);
        peers = std::move(in.peers);
        for (size_t index = 0; index < peers.size(); ++index) {
            auto* src = peers[index];
            if (src) {
                src->peer = this;
                src->peer_index = index;
            }
        }
        in.peers.clear();
    }

    Referable(const Referable& in) = delete;
    Referable& operator=(const Referable&) = delete;

    void AddRef(RefBase<Referable<T>>* ref)
    {
    //    printf("obj %p inserts %p\n", this, ref);
        if (!ref) {
            return;
        }
        if (ref->peer_index < peers.size() && peers[ref->peer_index] == ref) {
            return;
        }
//                ++allocated;
//                if (allocated % 10000 == 0) {
//                    std::print(stderr, "- {} -\n", allocated);
//                }
        ref->peer_index = peers.size();
        peers.push_back(ref);
    }

    void SubRef(RefBase<Referable<T>>* ref)
    {
    //    printf("obj %p removes %p\n", this, ref);
        if (!ref || peers.empty()) {
            return;
        }
        size_t index = ref->peer_index;
        if (index >= peers.size() || peers[index] != ref) {
            auto it = std::find(peers.begin(), peers.end(), ref);
            if (it == peers.end()) {
                ref->peer_index = std::numeric_limits<size_t>::max();
                return;
            }
            index = static_cast<size_t>(it - peers.begin());
        }
        size_t last = peers.size() - 1;
        if (index != last) {
            peers[index] = peers[last];
            peers[index]->peer_index = index;
        }
        peers.pop_back();
        ref->peer_index = std::numeric_limits<size_t>::max();
    }

    std::vector<RefBase<Referable<T>>*>& getPeers()
    {
        return peers;
    }

    // Move registered references atomically so bulk net rewiring remains linear in fanout.
    void movePeersTo(Referable<T>& destination, RefBase<Referable<T>>* keep = nullptr)
    {
        if (&destination == this) {
            return;
        }
        destination.peers.reserve(destination.peers.size() + peers.size());
        bool kept = false;
        for (RefBase<Referable<T>>* ref : peers) {
            if (ref == keep) {
                kept = true;
                continue;
            }
            ref->peer = &destination;
            ref->peer_index = destination.peers.size();
            destination.peers.push_back(ref);
        }
        peers.clear();
        if (kept) {
            keep->peer = this;
            keep->peer_index = 0;
            peers.push_back(keep);
        }
    }

    ~Referable()
    {
        for (auto* src : peers) {
            if (src) {
//            --allocated;
                src->peer = nullptr;
                src->peer_index = std::numeric_limits<size_t>::max();
            }
        }
    }
};

// it is very recommended not to set value of Ref in initialization by rvalue and better to use vector::reserve() before

template<class T>
struct Ref: public RefBase<Referable<T>>
{
    Ref() { }

    Ref(Ref&& in)  // in STL structs it will rewrite to Ref's unordered_set/vector twice when using emplace(obj)
    {
    //    printf("move %p from %p(%p)\n", this, &in, in.ref);
        set(in.peer);
        in.clear();
    }

    Ref(const Ref& in) = delete;
    Ref& operator=(const Ref&) = delete;

    void set(Referable<T>* setref)
    {
        clear();
        RefBase<Referable<T>>::peer = setref;
        if (setref) {
            setref->AddRef(this);
        }
    //    printf("ref %p set(%p)\n", this, setref);
    }

    Referable<T>* get()
    {
        return RefBase<Referable<T>>::peer;
    }

    void clear()
    {
        if (RefBase<Referable<T>>::peer) {
            RefBase<Referable<T>>::peer->SubRef(this);
            RefBase<Referable<T>>::peer = nullptr;
            RefBase<Referable<T>>::peer_index =
                std::numeric_limits<size_t>::max();
        //    printf("ref %p clear\n", this);
        }
    }

    Referable<T>* operator ->()
    {
        return RefBase<Referable<T>>::peer;
    }

    Referable<T>& operator *()
    {
        return *RefBase<Referable<T>>::peer;
    }

    ~Ref()
    {
        clear();
    }

    static Ref& fromBase(RefBase<Referable<T>>& base)
    {
        return static_cast<Ref&>(base);
    }

    static Ref* fromBase(RefBase<Referable<T>>* base)
    {
        return static_cast<Ref*>(base);
    }
};
