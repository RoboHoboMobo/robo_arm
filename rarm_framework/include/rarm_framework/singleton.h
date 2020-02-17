#ifndef SINGLETON_H
#define SINGLETON_H

template<class T>
class SingletonDestroyer
{
private:
    T* p_instance;
public:
    ~SingletonDestroyer();
    void initialize(T* p);
};

template<class T>
class Singleton
{
private:
    static T* p_instance;
    static SingletonDestroyer<T> destroyer;
protected:
    Singleton() { }
    Singleton(const Singleton&);
    Singleton& operator=(const Singleton&);
   ~Singleton() { }
    friend class SingletonDestroyer<T>;
public:
    static T& getInstance();
};

#endif // SINGLETON_H
