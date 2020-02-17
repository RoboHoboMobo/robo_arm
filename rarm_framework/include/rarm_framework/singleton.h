#ifndef SINGLETON_H
#define SINGLETON_H

class Controller_rarm
{
//public:
//  Controller_rarm();
//  virtual ~Controller_rarm();
};

class SingletonDestroyer
{
private:
    Controller_rarm* p_instance;
public:
    ~SingletonDestroyer();
    void initialize(Controller_rarm* p);
};

class Singleton
{
private:
    static Controller_rarm* p_instance;
    static SingletonDestroyer destroyer;
protected:
    Singleton() { }
    Singleton(const Singleton&);
    Singleton& operator=(const Singleton&);
   ~Singleton() { }
    friend class SingletonDestroyer;
public:
    static Controller_rarm& getInstance();
};

#endif // SINGLETON_H
