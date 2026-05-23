#ifndef INC_STATIC_VECTOR_H_
#define INC_STATIC_VECTOR_H_

#include <algorithm>
#include <initializer_list>
#include <cassert>

template<typename T, unsigned int N>
class StaticVector {
    private:
        T content[N];
        unsigned int sz = 0;
        unsigned int cap = N;

    public:
        StaticVector() {}

        StaticVector(std::initializer_list<T> init) {
            if((unsigned int)init.size() <= this->cap) {
                std::copy(init.begin(), init.end(), this->content);
                sz = init.size();
            }
        }

        StaticVector(T* content, unsigned int size) {
        	this->cap= size;
        	this->sz= size;
        	this->content= content;
        }

        StaticVector(T* content, unsigned int size, unsigned int capacity) {
        	if (size <= capacity) {
        		this->cap= capacity;
        		this->sz= size;
        		this->content= content;
        	}
        }

        // These 4 methods are what was missing — required for range-for loops
        T*       begin()       { return content; }
        T*       end()         { return content + sz; }
        const T* begin() const { return content; }
        const T* end()   const { return content + sz; }

        void push_back(T data) {
            if (sz < cap) content[sz++] = data;
        }

        void push_range(const T* data, unsigned int n) {
            if (sz + n <= cap) {
				for (unsigned int i = 0; i < n; i++)
					content[sz++] = data[i];
            }
        }

        void push_range(T* data, unsigned int dataSize) {
        	if (sz + dataSize <= cap){
				for (unsigned int i = 0; i < dataSize; i++)
					content[sz++] = data[i];
          	}
        }

        void remove(T data) {
            int index = -1;
            for (int i = 0; i < (int)sz; i++) {
                if (content[i] == data) { index = i; break; }
            }
            if (index >= 0) {
                sz--;
                for (int i = index; i < (int)sz; i++)
                    content[i] = content[i + 1];
            }
        }

        void removeAt(int index) {
            if (index >= 0 && index < (int)sz) {
                for (int i = index; i < (int)sz - 1; i++)
                    content[i] = content[i + 1];
                sz--;
            }
        }

        // Required by removeObserver in EventsManagement.h
        void erase(T* first, T* last) {
            if (first >= last) return;
            T* new_end = std::move(last, end(), first);
            sz = new_end - content;
        }

        void clear() { sz = 0; }

        T pop() {
            T head = content[0];
            removeAt(0);
            return head;
        }

        T& operator[](int index) {
            assert(index >= 0 && index < (int)sz);
            return content[index];
        }

        StaticVector<T, N> subVec(unsigned int startIndex, unsigned int stopIndex, bool isReverse) {
            if (startIndex >= 0 && stopIndex <= sz) {
				StaticVector<T, N> result= {};
				if (isReverse) {
					for (unsigned int i = stopIndex - 1; i >= startIndex; i--)
						result.push_back(content[i]);
				} else {
					for (unsigned int i = startIndex; i < stopIndex; i++)
						result.push_back(content[i]);
				}
				return result;
            }
			else {
				return {};
			}
        }

        const T* data()  const { return content; }
        T*       mutData()     { return content; }
        unsigned int      size()  const { return sz; }
        unsigned int capacity() { return cap; }

        StaticVector<T, N> copy() {
            StaticVector<T, N> copyVec(this->cap);
            copyVec.push_range(this->content, this->sz);  // was: size (method ptr), now: sz (value)
            return copyVec;
        }

        bool equals(T* other, unsigned int otherSize) {
        	if (this->sz == otherSize) {
        		for (unsigned int i= 0; i < this->sz; i++) {
        			if (this->content[i] != other[i]) {
        				return false;
        			}
        		}
        		return true;
        	} else {
        		return false;
        	}
        }

        int contains(T* other, unsigned int otherSize) {
        	return contains(other, otherSize, 0, this->sz - otherSize);
        }

        int contains(T* other, unsigned int otherSize, unsigned int startIndex) {
        	return contains(other, otherSize, startIndex, this->sz - otherSize);
        }

        int contains(T* other, unsigned int otherSize, unsigned int startIndex, unsigned int stopIndex) {
        	int status= -1;
            if (otherSize <= this->sz && startIndex <this->sz && startIndex <= stopIndex) {
            	if (stopIndex > this->sz - otherSize) {
            		stopIndex= this->sz - otherSize;
            	}
            	for (unsigned int i= startIndex; i < this->sz-otherSize; i++) {
            		status= i;
                    for (unsigned int j= 0; j < otherSize; j++) {
                    	if (this->content[i+j] != other[j]) {
                    		status= -1;
                    		break;
                    	}
                    }
                    if (status >= 0) {
                    	return status;
                    }
            	}
            }
            return status;
        }

        ~StaticVector() {}
};

#endif /* INC_STATIC_VECTOR_H_ */
