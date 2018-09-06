pipeline {
    agent { dockerfile true }
    stages {
        stage('test') {
            steps {
                sh '''
                . /robot/devel/setup.sh
                catkin_make test
                '''
            }
        }
    }
}
