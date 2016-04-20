#include "maxcpp/maxcpp6.h"

#include <vector>
#include <cmath>
#include <string>

#include <ext_buffer.h>

class Droneloop : public MspCpp6<Droneloop> {
public:

    enum State { PREBUFFER, IDLE, FADEOUT, LOOP, RECORD, RECORD_FADEOUT, FIRST_LOOP, ANY };
    
    enum Event { TAP, RESET, DONE, XFADE_CHANGED, NOP };

    typedef bool(Droneloop::*GuardFunction)();
    typedef void(Droneloop::*ActionFunction)();

    struct Transition {
        State source;
        Event event;
        State target;
        GuardFunction guard;
        ActionFunction action;
    };
    
    const char* stateLabel(State state) {
        switch (state) {
            case PREBUFFER:
                return "PREBUFFER";
                break;
            case IDLE:
                return "IDLE";
                break;
            case FADEOUT:
                return "FADEOUT";
                break;
            case LOOP:
                return "LOOP";
                break;
            case RECORD_FADEOUT:
                return "RECORD_FADEOUT";
                break;
            case RECORD:
                return "RECORD";
                break;
            case FIRST_LOOP:
                return "FIRST_LOOP";
                break;
            case ANY:
                return "ANY";
                break;
        }
        return "UNKNOWN";
    }

    const char* eventLabel(Event event) {
        switch (event) {
            case TAP:
                return "TAP";
                break;
            case RESET:
                return "RESET";
                break;
            case DONE:
                return "DONE";
                break;
            case XFADE_CHANGED:
                return "XFADE_CHANGED";
                break;
            case NOP:
                return "NOP";
                break;
        }
        return "UNKNOWN";
    }

    // guards
    bool nopGuard() {
        return true;
    }
    
    bool canFinishRecording() {
        // enough samples for xfade recorded?
        if (loop_pos_ < xfade_length_) {
            return false;
        }
        return true;
    }
    
    // end guards

    // actions
    void nopAction() {
    }

    void savePreBuffer() {
        int half_xfade = (xfade_length_/2);
        std::copy(&pre_buffer_[pre_buffer_pos_], &pre_buffer_[half_xfade], &fadein_part_[0]);
        std::copy(&pre_buffer_[0], &pre_buffer_[pre_buffer_pos_], &fadein_part_[half_xfade-pre_buffer_pos_]);
        loop_pos_ = 0;
    }
    
    void preFirstLoop() {
        loop_length_ = loop_pos_;
        loop_pos_ = 0;
    }
    
    void preLoop() {
        
//        t_buffer_obj *loop_buffer = buffer_ref_getobject(loop_buffer_ref_);
//        float* buffer_samples = buffer_locksamples(loop_buffer);
//        if (buffer_samples) {
//            std::fill(&buffer_samples[loop_pos_], &buffer_samples[buffer_length_], 0.0f);
//            buffer_unlocksamples(loop_buffer);
//            buffer_setdirty(loop_buffer);
//        }
        
        loop_pos_ = 0;
    }
    
    void preFadeout() {
        fadeout_start_ = loop_pos_;
        fadeout_pos_ = fadeout_start_;
    }

    void preRecordFadeout() {
        preFadeout();
        savePreBuffer();
    }
    
    void preIdle() {
        
        t_buffer_obj *loop_buffer = buffer_ref_getobject(loop_buffer_ref_);
        float* buffer_samples = internal_buffer_.data();
        if (internal_buffer_.size() == 0) {
            buffer_samples = buffer_locksamples(loop_buffer);    
        }
        
        if (buffer_samples) {
            std::fill(&buffer_samples[0], &buffer_samples[buffer_length_], 0.0f);
            if (internal_buffer_.size() == 0) {
                buffer_unlocksamples(loop_buffer);
            }
            buffer_setdirty(loop_buffer);
        }
        
        loop_pos_ = 0;
        loop_length_ = 0;
    }
    
    void setXFade() {
        xfade_time_ = xfade_time_scheduled_;
        xfade_length_ = int(samplerate_*xfade_time_/1000.0);
        xfade_length_ = 2*(std::ceil(xfade_length_/2.0)); // force multiple of 2
        
        pre_buffer_pos_ = 0;
        num_prebuffered_ = 0;
        
        preIdle();
    }
    
    // end actions
    
    static const Transition transitionTable_[13];
    
	Droneloop(t_symbol * sym, long ac, t_atom * av) {
        
        t_object* obj = &static_cast<t_pxobject&>(*this).z_ob;
        
        state_out_ = intout(obj);
        setupIO(1, 2);
        
        event_ = NOP;
        
        loop_buffer_ref_ = 0;
        
        xfade_time_scheduled_ = 200;
        xfade_time_ = xfade_time_scheduled_; // ms
        
        for (int i = 0; i < (ac > 2 ? 2 : ac); i++) {
            if ((av + i)->a_type == A_LONG) {
                xfade_time_ = atom_getlong(av+i);
            } else if ((av + i)->a_type == A_FLOAT) {
            } else if ((av + i)->a_type == A_SYM) {
                loop_buffer_ref_ = buffer_ref_new (obj, atom_getsym(av+i));
            } else {
                object_error(obj, "forbidden argument");
            }
            
        }
        
        if (loop_buffer_ref_ == 0) {
            object_warn(obj, "Buffer name missing.");
        }
        else if (! buffer_ref_exists(loop_buffer_ref_)) {
            object_warn(obj, "Invalid buffer name.");
        }

        post("droneloop~ 0.1 (c) Martin Gasser");
    }
	
	~Droneloop() {
        object_free(loop_buffer_ref_);
	}
	
	void bang(long inlet) {
        event_ = TAP;
    }
    
    void reset(long inlet) {
        event_ = RESET;
    }
    
    void xfade(long inlet, long ms) {
        if (ms >= 0 && ms < 2000) {
            xfade_time_scheduled_ = ms;
            event_ = XFADE_CHANGED;
        }
    }
    
    void buffername(long inlet, t_symbol* s) {
        t_object* obj = &static_cast<t_pxobject&>(*this).z_ob;
        loop_buffer_ref_ = buffer_ref_new (obj, s);
        
        if (buffer_ref_exists(loop_buffer_ref_)) {
            t_buffer_obj *loop_buffer = buffer_ref_getobject(loop_buffer_ref_);
            buffer_length_ = buffer_getframecount(loop_buffer);
            
            object_post(obj, "Buffer set to %s, length %d", s->s_name, buffer_length_);
        }

    }
    
    long notify(t_symbol *s, t_symbol *msg, void *sender, void *data) {
        return buffer_ref_notify(loop_buffer_ref_, s, msg, sender, data);
    }
    
    void assist(void *b, long m, long a, char *s) {
        if (m == ASSIST_INLET) {
            sprintf(s,"(signal) Input signal, (bang) start/stop recording, (reset) reset looper");
        }
        else {
            switch (a) {
                case 0:
                    sprintf(s, "(signal) Output signal");
                    break;
                case 1:
                    sprintf(s, "(signal) Loop sample index");
                    break;
                case 2:
                    sprintf(s, "(int) Looper state");
                    break;
            }
        }
    }
    
    float xfade_sample(float* samples, int pos) {
        int half_xfade = xfade_length_/2;
        int a = half_xfade;
        int b = loop_length_ - half_xfade;
        
        if ((pos >= 0) && (pos < a)) {
            int fade_pos = pos + half_xfade;
            // 0.5 .. 1.0
            double alpha = fade_pos/double(xfade_length_);
            double weight1 = std::sqrt(alpha);
            double weight2 = std::sqrt(1.0-alpha);
            return samples[pos]*weight1 + fadeout_part_[pos]*weight2;
        }
        else if ( (pos >= a) && (pos < b)) {
            return samples[pos];
        }
        else if ((pos >= b) && (pos < loop_length_)) {
            int fade_pos = pos-b;
            // 0.0 .. 0.5
            double alpha = fade_pos/double(xfade_length_);
            double weight1 = std::sqrt(alpha);
            double weight2 = std::sqrt(1.0-alpha);
            return fadein_part_[fade_pos]*weight1 + samples[pos]*weight2;
        }
        return 0.0;
    }

    void perform(double **ins, long numins, double **outs, long numouts, long sampleframes) {

        double * in = ins[0];
        double * out = outs[0];
        double * counter = outs[1];
        
        t_buffer_obj *loop_buffer = buffer_ref_getobject(loop_buffer_ref_);
        float* buffer_samples = internal_buffer_.data();

        if (internal_buffer_.size() == 0) {
            buffer_samples = buffer_locksamples(loop_buffer);
        }
        
        if (! buffer_samples) {
            for (int i=0; i < sampleframes; i++) {
                out[i] = 0.0;
                counter[i] = -1;
            }
            
            t_object* obj = &static_cast<t_pxobject&>(*this).z_ob;
            
            object_post(obj, "Invalid buffer.");
            return;
        }
        
        bool updated = false;
        
        for (int i=0; i < sampleframes; i++) {
            // always record the incoming signal into ring buffer
            if (xfade_length_ > 0) {
                pre_buffer_[pre_buffer_pos_] = in[i];
                pre_buffer_pos_ = (pre_buffer_pos_+1) % (xfade_length_/2);
                if (num_prebuffered_ < (xfade_length_/2)) {
                    num_prebuffered_++;
                }
            }

            // process state transition table
            for (int s = 0; s < 13; s++) {
                if ( ((transitionTable_[s].source == current_state_) || (transitionTable_[s].source == ANY)) && (transitionTable_[s].event == event_) ) {
                    GuardFunction guard = transitionTable_[s].guard;
                    if ((this->*guard)()) {
                        ActionFunction action = transitionTable_[s].action;
                        (this->*action)();
                        current_state_ = transitionTable_[s].target;
                        // consume event
                        event_ = NOP;
                        // post("Transition from %s to %s (%s)", stateLabel(transitionTable_[s].source), stateLabel(transitionTable_[s].target), eventLabel(transitionTable_[s].event) );
                        outlet_int(state_out_, current_state_);
                    }
                }
            }
            
            switch (current_state_) {
                case PREBUFFER:
                    if (num_prebuffered_ == (xfade_length_/2) || (xfade_length_ == 0)) {
                        event_ = DONE;
                    }
                    out[i] = 0.0;
                    break;
                case IDLE:
                    out[i] = 0.0;
                    break;
                case FADEOUT:
                {
                    if (xfade_length_ > 0) {
                        //  fade out loop
                        int diff = fadeout_pos_-fadeout_start_;
                        if (diff <= xfade_length_) {
                            double alpha = diff/double(xfade_length_);
                            out[i] = (1.0-alpha) * xfade_sample(buffer_samples, fadeout_pos_%loop_length_);
                            fadeout_pos_++;
                        }
                        
                        if (diff == xfade_length_) {
                            event_ = DONE;
                        }
                    }
                    else {
                        event_ = DONE;
                    }
                    break;
                }
                case RECORD:
                {
                    buffer_samples[loop_pos_] = in[i];
                    updated = true;
                    out[i] = 0.0;
                    if (++loop_pos_ == buffer_length_) {
                        event_ = TAP;
                    }
                    break;
                }
                case RECORD_FADEOUT:
                {
                    if (xfade_length_ > 0) {
                        // fade out loop
                        int diff = fadeout_pos_-fadeout_start_;
                        if (diff < xfade_length_) {
                            double alpha = diff/double(xfade_length_);
                            out[i] = (1.0-alpha) * xfade_sample(buffer_samples, fadeout_pos_%loop_length_);
                            fadeout_pos_++;
                        }
                        else {
                            out[i] = 0.0;
                        }
                    }
                    
                    buffer_samples[loop_pos_] = in[i];
                    updated = true;
                    if (++loop_pos_ == buffer_length_) {
                        event_ = TAP;
                    }
                    break;
                }
                case FIRST_LOOP:
                {
                    int a = (xfade_length_/2);
                    if ((loop_pos_ >= 0) && (loop_pos_ < a)) {
                        // alpha_ = 0.0 .. 1.0
                        // do a simple fade in and record the input signal into fadeout_part_ for later reuse
                        double alpha = loop_pos_/double(a);
                        out[i] = buffer_samples[loop_pos_]*alpha;
                        fadeout_part_[loop_pos_] = in[i];
                    }
                    else {
                        out[i] = xfade_sample(buffer_samples, loop_pos_);
                    }
                    if (++loop_pos_ == loop_length_) {
                        event_ = DONE;
                    }
                    break;
                }
                case LOOP:
                {
                    out[i] = xfade_sample(buffer_samples, loop_pos_);
                    loop_pos_ = (loop_pos_+1)%loop_length_;
                    break;
                }
            }
            
            counter[i] = loop_pos_/float(loop_length_);
        }

        if (internal_buffer_.size() == 0) {
            buffer_unlocksamples(loop_buffer);
            
            if (updated) {
                buffer_setdirty(loop_buffer);
            }
        }
    }
    

    
	void dsp(t_object * dsp64, short *count, double samplerate, long maxvectorsize, long flags) {
        
        t_object* obj = &static_cast<t_pxobject&>(*this).z_ob;

        samplerate_ = samplerate;
        
        xfade_length_ = int(samplerate_*xfade_time_/1000.0);
        xfade_length_ = 2*(std::ceil(xfade_length_/2.0)); // force multiple of 2
        
        int max_xfade_length = 2*samplerate_; // 2sec max xfade
        pre_buffer_.resize(max_xfade_length/2);
        fadein_part_.resize(max_xfade_length/2);
        fadeout_part_.resize(max_xfade_length/2);
        
        if (buffer_ref_exists(loop_buffer_ref_)) {
            t_buffer_obj *loop_buffer = buffer_ref_getobject(loop_buffer_ref_);
            buffer_length_ = buffer_getframecount(loop_buffer);
            internal_buffer_.resize(0);
        }
        else {
            object_warn(obj, "No buffer supplied, using internal buffer.");
            internal_buffer_.resize(30.0*samplerate); // 30s buffer default
        }
        
        pre_buffer_pos_ = 0;
        num_prebuffered_ = 0;
        
        loop_length_ = 0;
        loop_pos_ = 0;
        
        fadeout_start_ = 0;
        fadeout_pos_ = 0;
        
        current_state_ = PREBUFFER;
        outlet_int(state_out_, current_state_);
        
		REGISTER_PERFORM(Droneloop, perform);
	}

private:
    void* state_out_;
    
    t_buffer_ref* loop_buffer_ref_;
    // long in_;
    // void* proxy1_;
    
    float xfade_time_;
    
    std::vector<float> pre_buffer_;
    std::vector<float> fadein_part_;
    std::vector<float> fadeout_part_;
    int xfade_length_;

    std::vector<float> internal_buffer_;
    
    int pre_buffer_pos_;
    int num_prebuffered_;
    
    int buffer_length_;
    int loop_length_;
    int loop_pos_;
    
    int fadeout_start_;
    int fadeout_pos_;
    
    Event event_;
    int current_state_;
    
    int xfade_time_scheduled_;
    double samplerate_;
    
};


const Droneloop::Transition Droneloop::transitionTable_[13] = {
    { PREBUFFER, DONE, IDLE, &Droneloop::nopGuard, &Droneloop::preIdle              },
    { IDLE,             TAP,    RECORD,         &Droneloop::nopGuard,           &Droneloop::savePreBuffer               },
    { IDLE,             RESET,  IDLE,           &Droneloop::nopGuard,           &Droneloop::preIdle                 },
    { LOOP,             TAP,    RECORD_FADEOUT, &Droneloop::nopGuard,           &Droneloop::preRecordFadeout        },
    { LOOP,             RESET,  FADEOUT,        &Droneloop::nopGuard,           &Droneloop::preFadeout              },
    { FADEOUT,          DONE,   IDLE,           &Droneloop::nopGuard,           &Droneloop::preIdle                 },
    { RECORD,           TAP,    FIRST_LOOP,     &Droneloop::canFinishRecording, &Droneloop::preFirstLoop            },
    { RECORD,           RESET,  IDLE,           &Droneloop::nopGuard,           &Droneloop::preIdle               },
    { FIRST_LOOP,       DONE,   LOOP,           &Droneloop::nopGuard,           &Droneloop::preLoop               },
    { FIRST_LOOP,       RESET,  FADEOUT,           &Droneloop::nopGuard,           &Droneloop::preFadeout          },
    { RECORD_FADEOUT,   RESET,  IDLE,           &Droneloop::nopGuard,           &Droneloop::preIdle },
    { RECORD_FADEOUT,   TAP,    FIRST_LOOP,   &Droneloop::canFinishRecording, &Droneloop::preFirstLoop },
    { ANY, XFADE_CHANGED, PREBUFFER, &Droneloop::nopGuard, &Droneloop::setXFade }
};


C74_EXPORT int main(void) {
	Droneloop::makeMaxClass("droneloop~");
	REGISTER_METHOD(Droneloop, bang);
    REGISTER_METHOD(Droneloop, reset);
    REGISTER_METHOD_LONG(Droneloop, xfade);
    REGISTER_METHOD_NOTIFY(Droneloop, notify);
    REGISTER_METHOD_ASSIST(Droneloop, assist);
    REGISTER_METHOD_DEFSYM(Droneloop, buffername);
}
