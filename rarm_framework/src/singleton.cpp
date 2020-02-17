#include "../include/rarm_framework/singleton.h"

Controller_rarm* Singleton::p_instance = 0;
SingletonDestroyer Singleton::destroyer;

SingletonDestroyer::~SingletonDestroyer()
{
  delete p_instance;
}

void SingletonDestroyer::initialize(Controller_rarm* p)
{
  p_instance = p;
}

Controller_rarm& Singleton::getInstance()
{
  if(!p_instance)
  {
    p_instance = new Controller_rarm();
    destroyer.initialize(p_instance);
  }

  return *p_instance;
}

/*
template<class T>
SingletonDestroyer<T>::~SingletonDestroyer()
{
  delete instance_ptr;
}

template<class T>
SingletonDestroyer<T>::initialize(T* t_ptr)
{
  instance_ptr = t_ptr;
}

template<class T>
static T* Singleton<T>::getInstance()
{
  if(!uniq_instance)
  {
    uniq_instance = new T;
    destroyer.initialize(uniq_instance);
  }

  return uniq_instance;
}

template<class T> T* Singleton::uniq_instance = NULL;
template<class T> SingletonDestroyer Singleton::destroyer;
*/
