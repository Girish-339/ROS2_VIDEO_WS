pipeline {
    agent any

    environment {
        DOCKER_COMPOSE_FILE = 'docker-compose.yml'
    }

    stages {
        stage('Checkout') {
            steps {
                git branch: 'main', url: 'https://github.com/Girish-339/ROS2_VIDEO_WS.git'
            }
        }
             stage('Clean Old Containers') {
            steps {
                sh 'docker rm -f mediamtx || true'
            }
        }

        stage('Build Docker Images') {
            steps {
                sh 'docker-compose build'
            }
        }

        stage('Deploy Containers') {
            steps {
                // Stop old containers if any
                sh 'docker-compose down || true'

                // Start containers
                sh 'docker-compose up -d'
            }
        }

        stage('Verify') {
            steps {
                sh 'docker ps'
                sh 'docker-compose logs -f ros2_all'
            }
        }
   
    }

    post {
        always {
            echo 'Pipeline finished'
        }
    }
}
