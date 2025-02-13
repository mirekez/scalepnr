#pragma once

#include <vector>
#include <algorithm>

template <class T> class Ref;
template <class T> class Referable;

template <class T>
struct RefBase
{
    T* peer = nullptr;
};

template <class T>
struct Referable: public T
{
private:
    std::vector<RefBase<Referable<T>>*> peers;
public:
    Referable() {
    }
    Referable(T&& in) : T(std::move(in)) {
    }
    Referable(Referable&& in) : T(std::move(in)) {
//    printf("obj %p move from %p\n", this, &in);
        peers = std::move(in.peers);
        for (auto* src : peers) {
            if (src) {
                src->peer = this;
            }
        }
    }

    Referable(const Referable& in) = delete;
    Referable& operator=(const Referable&) = delete;

    void AddRef(RefBase<Referable<T>>* ref)
    {
    //    printf("obj %p inserts %p\n", this, ref);
        peers.push_back(ref);
    }

    void SubRef(RefBase<Referable<T>>* ref)
    {
    //    printf("obj %p removes %p\n", this, ref);
        auto it = std::find(peers.begin(), peers.end(), ref);
        if (it != peers.end()) {
            *it = nullptr;  // we keep element to let clear Referable peers during iteration over them, just make it zero
        }
    }

    std::vector<RefBase<Referable<T>>*>& getPeers()
    {
        peers.erase(std::remove_if(peers.begin(), peers.end(), [](auto ptr) { return ptr == nullptr; }), peers.end());
        return peers;
    }

    ~Referable()
    {
        for (auto* src : peers) {
            if (src) {
                src->peer = nullptr;
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

