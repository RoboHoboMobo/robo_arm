#include "rarm_framework/singleton.h"

template<class T>
SingletonDestroyer<T>::~SingletonDestroyer()
{
  delete p_instance;
}

template<class T>
void SingletonDestroyer<T>::initialize(T* p)
{
  p_instance = p;
}

template<class T>
T& Singleton<T>::getInstance()
{
  if(!p_instance)
  {
    p_instance = new T();
    destroyer.initialize(p_instance);
  }

  return *p_instance;
}

template<class T>
T* Singleton<T>::p_instance = 0;

template<class T>
SingletonDestroyer<T> Singleton<T>::destroyer;
