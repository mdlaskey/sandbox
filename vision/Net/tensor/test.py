import net2

net = net2.NetTwo()
sess = net.load(var_path='net2/net2_01-21-2016_02h14m08s.ckpt')
sess.close()
#net.optimize(100, path='net3/net3_01-19-2016_00h47m49s.ckpt')
