#include <Arduino.h>


class Knob 
{
    public:
        Knob(uint8_t prevAB, uint8_t initialcurRotVal)
            : _curRotVal(initialcurRotVal),_prevAB(prevAB),_min(0),_max(120),_prevfunc(0),_prevClick(1),_curClickVal(0){}
    void UpdateRotateVal (uint8_t curAB){
        switch (curAB | _prevAB<<2){
            case 0x01 : increment(); break;
            case 0x03 : impossibleState(); break;
            case 0x04 : decrement(); break;
            case 0x06 : impossibleState(); break;
            case 0x0B : decrement(); break;
            case 0x09 : impossibleState(); break;
            case 0x0E : increment(); break;
            case 0x0C : impossibleState(); break;
            default: break;
        }
        _prevAB = curAB;
    }

    void UpdateClick (bool curclick){
        if (curclick == 0 && _prevClick == 1){
            if (_curClickVal == 0){
                _curClickVal = 1;
            }
            else{
                _curClickVal = 0;
            }
        }
        _prevClick = curclick;

    }

 
    void SetLimits (uint8_t min, uint8_t max){
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

    bool curClickVal(){
        return _curClickVal;
    }

    private:
        int8_t _prevfunc;
        uint8_t _min;
        uint8_t _max;
        uint8_t _curRotVal;
        uint8_t _prevAB;
        bool _prevClick;
        bool _curClickVal;

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



