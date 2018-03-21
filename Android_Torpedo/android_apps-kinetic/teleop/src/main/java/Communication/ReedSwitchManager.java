package Communication;

import android.os.SystemClock;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by Nick Skupien on 3/20/2018.
 */

public class ReedSwitchManager {

    public interface OnClickListener {
        void onClick(View view);
    }

    public interface OnTouchListener {
        void onTouch(View view, MotionEvent motionEvent);
    }

    private boolean currState;
    private boolean prevState;

    private View navButton;

    private OnTouchListener onTouchListener;
    private OnClickListener onClickListener;

    public ReedSwitchManager(View navButton) {
        this.navButton = navButton;

        this.onClickListener = null;
        this.onTouchListener = null;
    }

    public void updateSwitchStates(boolean newState) {
        this.currState = newState;
        MotionEvent motionEvent;

        // check if switches were clicked
        if(this.onClickListener != null) {
            if (this.currState && !this.prevState) {
                this.onClickListener.onClick(navButton);
            }
        } else {
            Log.d("ROV_ERROR", " onClick listener for navigation buttons never set!");
        }

        // set on touch
        if(this.onTouchListener != null) {
            boolean released = (this.prevState && !this.currState);
            boolean clicked = (this.currState && !this.prevState);
            if(released || clicked) {
                int frontMotionEvent = 0;

                if (released) {
                    frontMotionEvent = MotionEvent.ACTION_UP;
                } else if (clicked) {
                    frontMotionEvent = MotionEvent.ACTION_DOWN;
                }

                motionEvent = MotionEvent.obtain(System.currentTimeMillis(), System.currentTimeMillis(), frontMotionEvent, 0, 0, 0);
                this.onTouchListener.onTouch(this.navButton, motionEvent);
            }
        } else {
            Log.d("ROV_ERROR", " OnTouch listener for navigation buttons never set!");
        }

        // update prev state
        this.prevState = this.currState;
    }

    public void setOnClickListener(OnClickListener oCL) {
        this.onClickListener = oCL;
    }

    public void setOnTouchListener(OnTouchListener oTL) {
        this.onTouchListener = oTL;
    }
}
