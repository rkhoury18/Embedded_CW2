#include <Arduino.h>


class Knob 
{
    public:
        Knob(uint8_t prevAB)
            : _curRotVal(4),_prevAB(prevAB),_min(-0xFF),_max(0xFF),_prevfunc(0){}
    void UpdateRotateVal (uint8_t curAB){
        switch (curAB | _prevAB<<2){
            case 0x01 : increment(); break;
            // case 0x03 : impossibleState(); break;
            case 0x04 : decrement(); break;
            // case 0x06 : impossibleState(); break;
            case 0x0B : decrement(); break;
            // case 0x0A : impossibleState(); break;
            case 0x0E : increment(); break;
            case 0x0C : impossibleState(); break;
            default: break;
        }
        _prevAB = curAB;
    }


    
    void SetLimits (int8_t min, int8_t max){
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
        int8_t _prevfunc;
        int8_t _min;
        int8_t _max;
        int8_t _curRotVal;
        uint8_t _prevAB;

        void increment (){
            if (_curRotVal < _max){
                _curRotVal++;
                _prevfunc = 1;
            }
        }
        void decrement (){
            if (_curRotVal > _min){
                _curRotVal--;
                _prevfunc = -1;
            }
        }
        void impossibleState (){
            if (_curRotVal < _max && _curRotVal > _min){
                _curRotVal+= _prevfunc;
            }
        }


};



