#ifndef INC_EVENTS_MANAGEMENT_H_
#define INC_EVENTS_MANAGEMENT_H_

#include <functional>
#include <memory>
#include <algorithm>
#include "StaticVector.h"

#define BROADCAST_ID 255

template <typename T>
class Observer {
    protected:
        std::function<void(T*)> callback;
        unsigned int id= BROADCAST_ID;
    public:
        Observer() {}
        Observer(unsigned int id) {
        	this->id= id;
        }
        virtual ~Observer() {}

        void respond(T* data) {
            callback(data);
        }

        void setCallback(std::function<void(T*)> cb) {
            this->callback = cb;
        }

        unsigned int getId() {
        	return this->id;
        }
};

template <typename T, unsigned int N>
class Publisher {
    private:
        StaticVector<std::shared_ptr<Observer<T>>, N> observers;
    public:
        Publisher() {
        }
        virtual ~Publisher() {}

        void trigger(T* data) {
        	for (int i= 0; i < observers.size(); i++) {
        		std::shared_ptr<Observer<T>> obs= observers[i];
        		obs->respond(data);
        	}
        }

        void trigger(T* data, unsigned int obsId) {
        	for (int i= 0; i < observers.size(); i++) {
        		if (obsId == observers[i]->getId()){
        			std::shared_ptr<Observer<T>> obs= observers[i];
					obs->respond(data);
               	}
            }
        }

        void addObserver(std::shared_ptr<Observer<T>> obs) {
            observers.push_back(obs);
        }

        void removeObserver(std::shared_ptr<Observer<T>> obs) {
            observers.erase(
                std::remove(observers.begin(), observers.end(), obs),
                observers.end()
            );
        }
};

#endif /* INC_EVENTS_MANAGEMENT_H_ */
