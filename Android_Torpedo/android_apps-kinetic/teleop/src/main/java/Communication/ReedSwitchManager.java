package Communication;

import android.os.SystemClock;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by Nick Skupien on 3/20/2018.
 */

public class ReedSwitchManager {

    public boolean front;
    public boolean center;
    public boolean rear;

    public boolean frontPrev;
    public boolean centerPrev;
    public boolean rearPrev;

    private View topNav;
    private View centerNav;
    private View bottomNav;

    public ReedSwitchManager(View topNav, View centerNav, View bottomNav) {
        this.topNav = topNav;
        this.centerNav = centerNav;
        this.bottomNav = bottomNav;
    }

    public void updateSwitchStates(boolean front, boolean center, boolean rear) {
        this.updateSwitchState(front, frontPrev, topNav);
        this.updateSwitchState(center, centerPrev, centerNav);
        this.updateSwitchState(rear, rearPrev, bottomNav );



    }

    private void updateSwitchState(boolean curr, boolean prev, View view) {
        // Obtain MotionEvent object
        long downTime = SystemClock.uptimeMillis();
        long eventTime = SystemClock.uptimeMillis() + 20;
        float x = 0.0f;
        float y = 0.0f;

        // List of meta states found here: developer.android.com/reference/android/view/KeyEvent.html#getMetaState()
        int metaState = 0;
        MotionEvent motionEvent = MotionEvent.obtain(
                downTime,
                eventTime,
                MotionEvent.ACTION_UP,
                x,
                y,
                metaState
        );

        // Dispatch touch event to view
        //view.dispatchTouchEvent(motionEvent);
    }
}
