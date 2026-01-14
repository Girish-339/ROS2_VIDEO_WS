pipeline {
    agent any

    options {
        disableConcurrentBuilds()
        timeout(time: 30, unit: 'MINUTES')
    }

    triggers {
        cron('15 14 * * *')   // Daily build at ~2 :05 PM
    }

    environment {
        COMPOSE_FILE = 'docker-compose.yml'
    }

    stages {

        stage('Checkout') {
            steps {
                git branch: 'main',
                    url: 'https://github.com/Girish-339/ROS2_VIDEO_WS.git'
            }
        }

        stage('Clean Existing Deployment') {
            steps {
                sh '''
                docker compose -f $COMPOSE_FILE down --remove-orphans || true
                docker rm -f mediamtx ros2_all || true
                '''
            }
        }

        stage('Build Images') {
            steps {
                sh '''
                docker compose -f $COMPOSE_FILE build
                '''
            }
        }

        stage('Deploy') {
            steps {
                sh '''
                docker compose -f $COMPOSE_FILE up -d
                '''
            }
        }

        stage('Verify Containers') {
            steps {
                sh '''
                docker ps
                docker compose ps
                docker compose logs ros2_all --tail=50
                '''
            }
        }
    }

    post {
        success {
            echo 'Deployment successful'
        }
        failure {
            echo 'Deployment failed'
        }
        always {
            sh 'docker system prune -f || true'
        }
    }
}
