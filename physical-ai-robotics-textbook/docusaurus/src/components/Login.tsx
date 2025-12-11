import React from 'react';
import Layout from '@theme/Layout';
import styles from './auth.module.css';

export default function Login() {
  return (
    <Layout
      title="Login"
      description="Login to Humanoid Robotics">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Welcome Back</h1>
          <p className={styles.authSubtitle}>Login to your account</p>
          
          <form className={styles.authForm}>
            <div className={styles.formGroup}>
              <label htmlFor="email">Email</label>
              <input 
                type="email" 
                id="email" 
                placeholder="Enter your email"
                required 
              />
            </div>
            
            <div className={styles.formGroup}>
              <label htmlFor="password">Password</label>
              <input 
                type="password" 
                id="password" 
                placeholder="Enter your password"
                required 
              />
            </div>
            
            <div className={styles.formOptions}>
              <label className={styles.checkbox}>
                <input type="checkbox" />
                <span>Remember me</span>
              </label>
              <a href="/forgot-password" className={styles.forgotLink}>
                Forgot password?
              </a>
            </div>
            
            <button type="submit" className={styles.authButton}>
              Login
            </button>
          </form>
          
          <div className={styles.authFooter}>
            <p>Don't have an account? <a href="/signup">Sign up</a></p>
          </div>
        </div>
      </div>
    </Layout>
  );
}