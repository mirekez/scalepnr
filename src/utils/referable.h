#pragma once

#include <vector>
#include <unordered_set>


struct RefBase
{
    void* ref = nullptr;
};

template <class T>
struct Referable: public T
{
    Referable() {
    }
    Referable(T&& in) : T(std::move(in)) {
    }
    Referable(Referable&& in) : T(std::move(in)) {
//    printf("obj %p move from %p\n", this, &in);
        dependencies = std::move(in.dependencies);
        for (auto* src : dependencies) {
            src->ref = this;
        }
    }

//    Referable& operator=(Referable&& in) {
////    printf("obj %p move from %p\n", this, &in);
//        dependencies = std::move(in.dependencies);
//        for (auto* src : dependencies) {
//            src->ref = this;
//        }
//    }

    Referable(const Referable& in) = delete;
    Referable& operator=(const Referable&) = delete;

    std::unordered_set<RefBase*> dependencies;

    void AddRef(RefBase* ref)
    {
    //    printf("obj %p inserts %p\n", this, ref);
        dependencies.insert(ref);
    }

    void SubRef(RefBase* ref)
    {
    //    printf("obj %p removes %p\n", this, ref);
        dependencies.erase(ref);
    }

    ~Referable()
    {
        for (auto* src : dependencies) {
            src->ref = nullptr;
        }
    }
};

// it is very recommended not to set value of Ref in initialization by rvalue

template<class T>
struct Ref: public RefBase
{
    Ref()
    {
    }

    Ref(Ref&& in)  // in STL structs it will write to Ref's unordered_set twice when using emplace(obj)
    {
    //    printf("move %p from %p(%p)\n", this, &in, in.ref);
        set(static_cast<Referable<T>*>(in.ref));
        in.clear();
    }

//    Ref& operator=(Ref&& in) {
//    //    printf("move %p from %p(%p)\n", this, &in, in.ref);
//        set(static_cast<Referable<T>*>(in.ref));
//        in.clear();
//    }

    Ref(const Ref& in) = delete;
    Ref& operator=(const Ref&) = delete;

    void set(Referable<T>* setref)
    {
        clear();
        ref = (void*)setref;
        if (setref) {
            setref->AddRef(this);
        }
    //    printf("ref %p set(%p)\n", this, setref);
    }

    void clear()
    {
        if (ref) {
            (static_cast<Referable<T>*>(ref))->SubRef(this);
            ref = nullptr;
        //    printf("ref %p clear\n", this);
        }
    }

    ~Ref()
    {
        clear();
    }
};

