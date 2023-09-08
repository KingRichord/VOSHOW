# VOSHOW
## A tool for viewing robot poses and point clouds using ZMQ

## Example ##
![](demo.gif)
The next example is how to use：

```c++
your robot:
    std::shared_ptr<Talker> m_talker_= std::make_shared<Talker>();
	std::string ip = "tcp://172.17.0.1:5555";
	m_talker_->Init(ip);
	
    m_talker_->PubKeyFrame(kfs);
    m_talker_->PubPose(Ts[0]);
your VOSHOW
    subscriber.connect("tcp://172.17.0.1:5555");
```
how to install 
``` bash
git submodule sync
git submodule init
git submodule update

mkdir build
cd build
cmake ..

```
## License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
We are still working on improving the code reliability. For any technical issues, please contact Yangyang liu <www.@qq837374741@foxmail.com>