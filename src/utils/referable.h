#pragma once

#include <vector>
#include <unordered_set>


struct RefBase
{
    void* ref = nullptr;
};


struct Referable
{
    std::unordered_set<RefBase*> dependencies;

    void addRef(RefBase* ref)
    {
        dependencies.insert(ref);
    }

    void subRef(RefBase* ref)
    {
        dependencies.erase(ref);
    }

    ~Referable()
    {
        for (auto *src : dependencies) {
            src->ref = nullptr;
        }
    }
};



template<class T>
struct Ref: public RefBase
{
    Ref()
    {
    }

    Ref(Ref&& in)  // in STL structs it will write to ref's unordered_set twice when using emplace(obj)
    {
        set((T*)in.ref);
        in.clear();
    }

    Ref(const Ref& in) = delete;
    Ref& operator=(const Ref&) = delete;

    void set(T* setRef)
    {
        if (!setRef) {
            clear();
            return;
        }
        ref = (void*)setRef;
        setRef->addRef(this);
    }

    void clear()
    {
        if (ref) {
            ((T*)ref)->subRef(this);
            ref = nullptr;
        }
    }

    ~Ref()
    {
        clear();
    }
};

