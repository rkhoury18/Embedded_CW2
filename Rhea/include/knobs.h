#include <Arduino.h>


class Knob 
{
    public:
        Knob(uint8_t prevAB)
            : _curRotVal(0),_prevAB(prevAB){}

    void UpdateRotateVal (uint8_t curAB){
        switch (curAB | _prevAB<<2){
            case 0x01: increment(); break;
            case 0x04: decrement(); break;
            case 0x0B : decrement(); break;
            case 0x0E : increment(); break;
            case 0x0C : increment(); break;
            default: break;
        }
        _prevAB = curAB;
    }


    
    void SetLimits (int min, int max){
        _min = min;
        _max = max;
        if (_curRotVal <= _min){
            _curRotVal = _min;
        }
        else if (_curRotVal >= _max){
            _curRotVal = _max;
        }

    }
    int8_t CurRotVal (){
        return _curRotVal;
    }
    private:
        int _min;
        int _max;
        int8_t _curRotVal;
        uint8_t _prevAB;

        void increment (){
            if (_curRotVal < _max){
                _curRotVal++;
            }
        }
        void decrement (){
            if (_curRotVal > _min){
                _curRotVal--;
            }
        }


};


