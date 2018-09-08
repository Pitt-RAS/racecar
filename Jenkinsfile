node {
    stage('Pre-build') {
        checkout scm
        sh 'git submodule update --init --recursive'
    }

    stage 'Build'
    def image = docker.build "magellan-2018-$BUILD_ID"

    stage('Test') {
        image.inside {
            sh '''
            . /robot/devel/setup.sh
            catkin_make
            catkin_make run_tests
            '''
        }
    }
    stage('Build Teensy') {
        image.inside {
            sh '''
            . /robot/devel/setup.sh
            rosrun rosserial_arduino make_libraries.py /root/Arduino/libraries/
            /robot/src/magellan_firmware/compile.sh
            '''
        }
    }
}
