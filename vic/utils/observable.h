#pragma once

namespace vic
{

// struct SomeObservableClass : public Observable<SomeObservableClass> {};
// SomeObservableClass::Observe();

// observable wrapper, using crtp
template <typename T>
class Observable
{
    // handles are only valid as long as Observable is not moved.
    // Observables can also not be moved while observed
    template <typename T2>
    class ObservableHandle
    {
    public:
        ObservableHandle(Observable& observable)
            : mObservable(observable)
        {
            mObservable.StartObserving();
        }

        // todo: handles should be movable, but not copyable
        ObservableHandle(const ObservableHandle&) = delete;
        ObservableHandle(ObservableHandle&&) = delete;
        ObservableHandle& operator=(const ObservableHandle&) = delete;
        ObservableHandle& operator=(ObservableHandle&&) = delete;

        ~ObservableHandle() { mObservable.StopObserving(); }

    private:
        Observable<T>& mObservable;
    };

    // todo: an observable should be movable/copyable, but only while not observed
    Observable(const Observable&) = delete;
    Observable(Observable&&) = delete;
    Observable& operator=(const Observable&) = delete;
    Observable& operator=(Observable&&) = delete;

public:
    Observable() = default;
    ObservableHandle<T> Observe() { return ObservableHandle<T>{*this}; }

    bool IsObserved() const { return mObserverCount > 0; }

private:
    std::size_t mObserverCount{0};

    // called by ObservableHandle
    void StartObserving() { mObserverCount++; }
    void StopObserving() { mObserverCount--; }
};

} // namespace vic