

def test(controller, izzy, turntable, bincam, show_bin=False, show_frame=False)
    """
    Test drive izzy with options to show camera
    frame or not. Note that not showing frames will
    greatly enhance control performance
    """
    while True:
        if show_bin:        bincam.read_binar_frame(show=True)
        elif show_frame:    bincam.read_frame(show=True)
        controls = controller.getUpdates()
        print "Test: " + str(controls)
        if controls is None:
            print "Done"
            izzy.stop()
            break
        controls[1] = 0
        controls[2] = 0
        
        

